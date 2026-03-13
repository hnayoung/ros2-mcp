import argparse
import json
import sys
import time
from typing import Any, Dict

from ros_mcp.client.mcp_client import MCPClient


NUM_BODY_JOINTS = 29
NUM_HAND_JOINTS = 6


def _parse_value(raw: str) -> Any:
    """가능하면 JSON 리터럴로 파싱하고, 실패하면 원문 문자열을 유지합니다."""
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        return raw


def _parse_key_value_args(items: list[str]) -> Dict[str, Any]:
    arguments: Dict[str, Any] = {}
    for item in items:
        if "=" not in item:
            raise ValueError(f"Invalid --arg value: {item!r}. Expected key=value.")
        key, value = item.split("=", 1)
        key = key.strip()
        if not key:
            raise ValueError(f"Invalid --arg value: {item!r}. Key cannot be empty.")
        arguments[key] = _parse_value(value)
    return arguments


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="LLM 없이 ROS MCP 도구를 직접 호출합니다."
    )
    parser.add_argument(
        "tool",
        nargs="?",
        help="호출할 도구 이름 (예: get_joint_states, send_joint_command).",
    )
    parser.add_argument(
        "--fn",
        default="",
        help="호출할 함수 이름(=tool). positional tool 대신 사용할 수 있습니다.",
    )
    parser.add_argument(
        "--joint-id",
        type=int,
        help="단축 모드: send_joint_command의 대상 관절 ID.",
    )
    parser.add_argument(
        "--q",
        type=float,
        help="단축 모드: send_joint_command의 목표 관절 위치(rad).",
    )
    parser.add_argument(
        "--dq",
        type=float,
        default=0.0,
        help="단축 모드: 목표 관절 속도(rad/s).",
    )
    parser.add_argument(
        "--tau",
        type=float,
        default=0.0,
        help="단축 모드: 피드포워드 토크(Nm).",
    )
    parser.add_argument(
        "--kp",
        type=float,
        default=50.0,
        help="단축 모드: 위치 게인.",
    )
    parser.add_argument(
        "--kd",
        type=float,
        default=1.0,
        help="단축 모드: 속도 게인.",
    )
    parser.add_argument(
        "--args",
        default="",
        help='도구 인자를 JSON 객체로 전달 (예: \'{"joint_id": 0, "q": 0.1}\').',
    )
    parser.add_argument(
        "--arg",
        action="append",
        default=[],
        help="key=value 형태의 단일 인자. 여러 번 지정 가능하며 값은 JSON 리터럴을 허용합니다.",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="사용 가능한 도구 목록을 출력하고 종료합니다.",
    )
    parser.add_argument(
        "--pretty",
        action="store_true",
        help="JSON 결과를 보기 좋게 출력합니다.",
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=0.0,
        help="명령 전송 후 지정한 초만큼 프로세스를 유지합니다.",
    )
    parser.add_argument(
        "--repeat",
        type=int,
        default=1,
        help="도구 호출 반복 횟수.",
    )
    parser.add_argument(
        "--repeat-forever",
        action="store_true",
        help="중단(Ctrl+C)할 때까지 도구를 계속 호출합니다.",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="명령을 1회 전송하고 종료합니다.",
    )
    parser.add_argument(
        "--hold",
        action="store_true",
        help="Ctrl+C 전까지 명령을 계속 전송해 자세를 유지합니다.",
    )
    parser.add_argument(
        "--interval-seconds",
        type=float,
        default=0.05,
        help="반복 호출 간 지연 시간(초).",
    )
    return parser


def _load_arguments(args_json: str, arg_items: list[str]) -> Dict[str, Any]:
    arguments: Dict[str, Any] = {}

    if args_json:
        try:
            parsed = json.loads(args_json)
        except json.JSONDecodeError as exc:
            raise ValueError(f"Invalid --args JSON: {exc}") from exc
        if not isinstance(parsed, dict):
            raise ValueError("--args must be a JSON object.")
        arguments.update(parsed)

    arguments.update(_parse_key_value_args(arg_items))
    return arguments


def _print_tool_list(client: MCPClient) -> None:
    for tool in client.available_tools:
        function = tool["function"]
        name = function["name"]
        description = function.get("description", "")
        print(name)
        if description:
            print(f"  {description}")


def _publish_zero_commands(client: MCPClient) -> None:
    """Ctrl+C로 중지할 때 마지막 자세가 유지되지 않도록 파라미터 값에 0을 넣음"""
    try:
        client.call_tool(
            "send_joint_positions",
            {
                "positions": [0.0] * NUM_BODY_JOINTS,
                "kp": 0.0,
                "kd": 0.0,
            },
        )
        client.call_tool(
            "send_hand_command",
            {
                "hand": "left",
                "positions": [0.0] * NUM_HAND_JOINTS,
                "kp": 0.0,
                "kd": 0.0,
            },
        )
        client.call_tool(
            "send_hand_command",
            {
                "hand": "right",
                "positions": [0.0] * NUM_HAND_JOINTS,
                "kp": 0.0,
                "kd": 0.0,
            },
        )
        # 서버 종료 전에 ROS discovery/전송이 완료될 짧은 시간을 둡니다.
        time.sleep(0.2)
    except Exception:
        # 안전 처리용 best-effort 경로이며, 주 오류 처리는 호출자에서 수행합니다.
        pass


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()

    client = MCPClient()
    try:
        client.start_server()

        if args.list:
            _print_tool_list(client)
            return 0

        if args.fn and args.tool and args.fn != args.tool:
            parser.error("tool과 --fn을 동시에 지정할 때는 같은 이름이어야 합니다.")

        tool_name = args.fn or args.tool
        shortcut_mode = args.joint_id is not None or args.q is not None
        if shortcut_mode:
            if args.joint_id is None or args.q is None:
                parser.error("--joint-id와 --q는 함께 사용해야 합니다.")
            if not tool_name:
                parser.error("단축 모드에서는 tool 또는 --fn으로 함수 이름을 지정해야 합니다.")
            arguments = {
                "joint_id": args.joint_id,
                "q": args.q,
                "dq": args.dq,
                "tau": args.tau,
                "kp": args.kp,
                "kd": args.kd,
            }
        else:
            if not tool_name:
                parser.error("--list를 사용하지 않는 경우 tool이 필요합니다.")
            arguments = _load_arguments(args.args, args.arg)

        if args.repeat < 1:
            parser.error("--repeat는 1 이상이어야 합니다.")
        if args.interval_seconds < 0:
            parser.error("--interval-seconds는 0 이상이어야 합니다.")

        if args.once and args.hold:
            parser.error("--once와 --hold는 함께 사용할 수 없습니다.")

        if args.hold:
            repeat_count = None
        elif args.once:
            repeat_count = 1
        elif args.repeat_forever:
            repeat_count = None
        else:
            repeat_count = args.repeat

        call_index = 0
        printed_once = False
        while repeat_count is None or call_index < repeat_count:
            call_index += 1
            result = client.call_tool(tool_name, arguments)

            should_print = not printed_once
            if should_print:
                try:
                    parsed_result = json.loads(result)
                except json.JSONDecodeError:
                    print(result)
                else:
                    print(json.dumps(parsed_result, indent=2, ensure_ascii=False))
                printed_once = True

            if repeat_count is not None and call_index >= repeat_count:
                break

            if args.interval_seconds > 0:
                time.sleep(args.interval_seconds)

        if args.hold_seconds > 0:
            time.sleep(args.hold_seconds)
        return 0
    except KeyboardInterrupt:
        _publish_zero_commands(client)
        return 130
    except ValueError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
