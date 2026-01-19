import asyncio
import json
import os
import select
import subprocess
from typing import List, Dict, Optional
import requests
import traceback
import time
from dotenv import load_dotenv

# .env 파일 로드
load_dotenv()

# OpenRouter API 설정
OPENROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY", "")
API_URL = "https://openrouter.ai/api/v1/chat/completions"
MODEL = "openai/gpt-oss-20b"  
MAX_TOKENS = 1024
TEMPERATURE = 0.7
MAX_AGENT_ITERATIONS = 10 # 최대 에이전트 반복 횟수

class MCPClient:
    """간단한 MCP 클라이언트 - subprocess 통신"""

    def __init__(self):
        self.server_process = None
        self.available_tools: List[Dict] = []
        self.request_id = 0

    def _send_request(self, method: str, params: Dict = None) -> Dict:
        """JSON-RPC 요청 전송 및 응답 수신"""
        self.request_id += 1
        message = {
            "jsonrpc": "2.0",
            "id": self.request_id,
            "method": method,
            "params": params or {}
        }

        self.server_process.stdin.write(json.dumps(message) + '\n')
        self.server_process.stdin.flush()
        response = self.server_process.stdout.readline()

        if not response.strip():
            # 빈 응답인 경우 stderr 확인
            stderr_output = ""
            if self.server_process.stderr:
                if select.select([self.server_process.stderr], [], [], 0.1)[0]:
                    stderr_output = self.server_process.stderr.read(4096)
            raise RuntimeError(f"서버로부터 빈 응답을 받았습니다. stderr: {stderr_output}")

        return json.loads(response)

    def start_server(self):
        """MCP 서버 시작"""
        robot_type = os.getenv("ROBOT_TYPE", "turtlesim")
        print(f"MCP 서버 시작 중... (ROBOT_TYPE={robot_type})")
        # 프로젝트 루트 디렉토리 계산 (client -> mcp -> src -> project_root)
        project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

        # 환경 변수를 서브프로세스에 전달
        env = os.environ.copy()
        env["ROS_LOG_DIR"] = "/tmp"

        self.server_process = subprocess.Popen(
            ['python3', '-m', 'ros_mcp.server'],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,  # stderr도 캡처하여 디버깅 가능
            text=True,
            bufsize=1,
            cwd=project_root,  # src 디렉토리에서 실행
            env=env  # 환경 변수 전달
        )

        # 서버 시작 대기
        time.sleep(0.5)

        # 초기화
        self._send_request("initialize", {
            "protocolVersion": "2024-11-05",
            "capabilities": {},
            "clientInfo": {"name": "MCPClient", "version": "1.0"}
        })

        # 도구 목록 가져오기
        response = self._send_request("tools/list")
        mcp_tools = response["result"]["tools"]

        for tool in mcp_tools:
            # 도구 호출 형식 변환 (OpenAI/Gemini 호환 형식)
            input_schema = tool.get("inputSchema", {})

            # Gemini는 parameters가 비어있으면 문제가 될 수 있으므로 기본 구조 보장
            if not input_schema:
                input_schema = {
                    "type": "object",
                    "properties": {},
                    "required": []
                }

            # type이 없으면 추가
            if "type" not in input_schema:
                input_schema["type"] = "object"

            tool_def = {
                "type": "function",
                "function": {
                    "name": tool.get("name"),
                    "description": tool.get("description", ""),
                    "parameters": input_schema
                }
            }
            self.available_tools.append(tool_def)


        print(f"MCP 서버 연결 완료 ({len(self.available_tools)}개 tool 사용 가능)\n")

    def call_tool(self, tool_name: str, arguments: Dict) -> str:
        """도구 호출"""
        result_data = self._send_request("tools/call", {
            "name": tool_name,
            "arguments": arguments
        })

        if "result" in result_data and "content" in result_data["result"]:
            content = result_data["result"]["content"]
            if isinstance(content, list):
                return "\n".join(
                    item.get("text", str(item)) for item in content
                )
            return str(content)

        return "Tool 실행 완료"

    def close(self):
        """서버 종료"""
        if self.server_process:
            self.server_process.terminate()
            self.server_process.wait()
        print("MCP 서버 종료")


# LLM 호출
def call_openrouter(messages: List[Dict], tools: Optional[List[Dict]] = None) -> Optional[Dict]:
    """OpenRouter API 호출"""
    if not OPENROUTER_API_KEY:
        print("[ERROR] OPENROUTER_API_KEY를 설정해주세요.")
        return None

    headers = {
        "Authorization": f"Bearer {OPENROUTER_API_KEY}",
        "Content-Type": "application/json",
        "HTTP-Referer": "http://localhost",
        "X-Title": "ROS MCP Client",
    }

    data = {
        "model": MODEL,
        "messages": messages,
        "max_tokens": MAX_TOKENS,
        "temperature": TEMPERATURE,
    }

    if tools:
        data["tools"] = tools
        data["tool_choice"] = "required"  # 강제로 tool calling 활성화

    try:
        response = requests.post(API_URL, headers=headers, json=data, timeout=60)
        response.raise_for_status()
        return response.json()
    except Exception as e:
        print(f"[ERROR] API 호출 오류: {e}")
        return None


# Agent Loop
def agent_loop(client: MCPClient, user_input: str):
    """에이전트 실행 루프 - 자동 위치 확인 및 재시도 기능 포함"""
    conversation_history = [
        {"role": "user", "content": user_input}
    ]

    print("생각 중...\n")

    for iteration in range(MAX_AGENT_ITERATIONS):
        print(f"--- Iteration {iteration + 1}/{MAX_AGENT_ITERATIONS} ---")
        print("[LLM] API 호출 중...")

        # LLM API 호출 시간 측정
        llm_start = time.time()
        response = call_openrouter(
            conversation_history,
            tools=client.available_tools
        )
        llm_elapsed = time.time() - llm_start

        if not response or "choices" not in response:
            print("[ERROR] LLM 응답 실패")
            break

        print(f"[LLM] 응답 시간: {llm_elapsed:.2f}초")

        message = response["choices"][0]["message"]


        conversation_history.append(message)


        if message.get("tool_calls"):
            print(f"[LLM] Tool calls: {len(message['tool_calls'])}개")
            for tool_call in message["tool_calls"]:
                function_name = tool_call["function"]["name"]
                raw_arguments = tool_call["function"]["arguments"]

                # JSON 파싱 시도
                try:
                    # 빈 문자열이면 바로 빈 dict 사용
                    if not raw_arguments or not raw_arguments.strip():
                        arguments = {}
                    else:
                        arguments = json.loads(raw_arguments)
                except json.JSONDecodeError:
                    # 잘못된 JSON 패턴 감지 및 수정
                    if raw_arguments.strip() in ['{""}', '{""}', '{" "}']:
                        arguments = {}
                    else:
                        arguments = {}

                # 함수 시그니처 스타일로 출력
                if arguments:
                    args_str = ", ".join(f"{k}={v}" for k, v in arguments.items())
                    print(f"[TOOL] {function_name}({args_str})")
                else:
                    print(f"[TOOL] {function_name}()")

                # 실행 시간 측정
                start_time = time.time()
                result = client.call_tool(function_name, arguments)
                elapsed_time = time.time() - start_time

                print(f"[RESULT] {result} (실행시간: {elapsed_time:.2f}초)")

                conversation_history.append({
                    "role": "tool",
                    "tool_call_id": tool_call["id"],
                    "content": result
                })

        else:
            # 최종 응답 출력
            content = message.get('content', '')

            if not content:
                # content가 비어있으면 에러로 처리하고 LLM에게 피드백
                error_msg = "ERROR: You must either call a tool or provide a text response. Empty responses are not allowed."
                print(f"[ERROR] LLM이 tool calling 없이 빈 응답을 반환했습니다.")
                print(f"[SYSTEM] LLM에게 에러 피드백 전송: {error_msg}")

                # LLM이 읽을 수 있도록 conversation history에 추가
                conversation_history.append({
                    "role": "user",
                    "content": error_msg
                })
                # 다음 iteration에서 재시도
                continue

            # 응답 내용 간략하게 미리보기
            preview = content[:100] + "..." if len(content) > 100 else content
            print(f"[LLM] Response: {preview}\n")
            break


# Main
def main():
    print("=" * 60)
    print("Simple ROS MCP Client")
    print("=" * 60)

    client = MCPClient()

    try:
        client.start_server()

        while True:
            user_input = input("You: ").strip()
            if user_input.lower() in ["exit", "quit", "q"]:
                break
            if user_input:
                agent_loop(client, user_input)

    except KeyboardInterrupt:
        print("\n[INFO] 사용자에 의해 중단되었습니다.")
    except Exception as e:
        print(f"[ERROR] 예상치 못한 오류 발생: {e}")
        traceback.print_exc()
    finally:
        client.close()


if __name__ == "__main__":
    main()
