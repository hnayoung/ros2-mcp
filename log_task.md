# ROS 2 MCP Server セットアップ作業ログ

**日時**: 2025年5月2日 17:46 (JST)

## 1. 初期状態の確認

- `requirements.txt`に`rclpy`と`fastmcp`が含まれていることを確認
- 現在のPythonバージョンが3.12であることを確認
- ROS 2 Humbleがインストールされていることを確認（`/opt/ros/humble/`）

## 2. uvの仮想環境へのパッケージインストール

- `fastmcp`パッケージのインストールを試行
  ```bash
  uv pip install -r requirements.txt
  ```
- `rclpy`パッケージがPyPIに存在しないためエラー発生
- `fastmcp`のみをインストール
  ```bash
  uv pip install fastmcp
  ```
- `rclpy`はROS 2のセットアップスクリプトをソースすることで使用可能

## 3. Python 3.10の設定

- ROS 2 HumbleはPython 3.10用にビルドされているため、Python 3.12では動作しない問題を確認
- `.python-version`ファイルを更新してPython 3.10を使用するように設定
  ```
  3.10
  ```
- 仮想環境を再作成
  ```bash
  rm -rf .venv
  uv venv --python /usr/bin/python3.10
  ```

## 4. プロジェクト設定の更新

- `pyproject.toml`ファイルのPythonバージョン要件を更新
  ```toml
  requires-python = ">=3.10"
  ```
- 依存関係を追加
  ```toml
  dependencies = [
      "fastmcp",
      "numpy",
  ]
  ```
- `requirements.txt`ファイルは不要なため削除（`pyproject.toml`で依存関係を管理）

## 5. MCPサーバー設定の更新

- Cline MCPサーバー設定を確認
- 作業ディレクトリとスクリプトのフルパスを明示的に指定
  ```json
  "ros2-mcp-server": {
    "autoApprove": [],
    "disabled": false,
    "timeout": 60,
    "command": "uv",
    "args": [
      "--directory",
      "/mnt/hdd/git/ros2-mcp-server",
      "run",
      "bash",
      "-c",
      "export ROS_LOG_DIR=/tmp && source /opt/ros/humble/setup.bash && python3 /mnt/hdd/git/ros2-mcp-server/ros2-mcp-server.py"
    ],
    "transportType": "stdio"
  }
  ```
- ROS 2のロギングディレクトリを`/tmp`に設定して、ロギングエラーを解決

## 6. 動作確認

- MCPサーバーを使用してロボットを制御
  ```json
  {
    "linear": [0.2, 0.0, 0.0],
    "angular": [0.0, 0.0, 0.0],
    "duration": 5.0
  }
  ```
- 「Successfully moved for 5.0 seconds and stopped」というレスポンスを確認

## 7. ドキュメント更新

- `README.md`ファイルを更新
  - uvの環境が必要なことを明記
  - Python 3.10が必要なことを明記
  - MCPサーバーの設定方法を詳細に説明
  - ファイル名やディレクトリ構造を更新
  - インストール手順を更新
  - 使用方法を更新
  - トラブルシューティングセクションを追加

## 8. まとめ

- Python 3.10を使用するuvの仮想環境を設定
- 必要なパッケージ（`fastmcp`と`numpy`）をインストール
- MCPサーバー設定を最適化
- ROS 2のロギングエラーを解決
- ドキュメントを更新して、セットアップと使用方法を明確化

これにより、ROS 2 MCPサーバーが正常に動作し、Claudeを通じてロボットを制御できるようになりました。

## 9. Gitリポジトリの設定

- `.gitignore`ファイルを更新して、`uv.lock`ファイルを追加
  ```
  # Package manager files
  uv.lock
  ```

## 10. ライセンスの設定

- MITライセンスを選択
- ライセンス情報をREADME.mdに記載
  ```
  MIT License

  Copyright (c) 2025 kakimochi

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  ...
  ```
- FastMCPのApache License 2.0の条件も適用されることを明記

## 11. README.mdの更新

- GitHubリポジトリのURLを修正
  ```bash
  git clone https://github.com/kakimochi/ros2-mcp-server.git
  ```
- ディレクトリ構造を更新して、`.gitignore`ファイルを含める
- Installationセクションを更新して、ROS 2 Humbleの制約でPython 3.10が必要なことを明記
- 仮想環境のアクティベーションに関する手順を追加
  ```bash
  source .venv/bin/activate
  ```
- Cline（VSCode拡張機能）の設定方法を追加
  - Clineアイコンをクリックして設定を開く
  - MCPサーバー設定セクションに移動
  - 設定内容を追加（Claude Desktopと同様の設定）
  - 設定後の操作（設定画面から直接ON/OFFや接続確認が可能）

## 12. コードの整理

- `ros2-mcp-server.py`のコードを整理して可読性を向上

## 13. 最終確認

- すべての設定が正しく行われていることを確認
- MCPサーバーが正常に動作することを確認
- ドキュメントが最新の状態であることを確認

これにより、ROS 2 MCPサーバーのセットアップが完了し、GitHubリポジトリとして公開する準備が整いました。
