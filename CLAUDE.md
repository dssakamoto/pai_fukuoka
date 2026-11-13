## 開発コマンド

### テスト実行
```bash
uv sync
uv run pytest            # すべてのテスト
uv run pytest --cov      # カバレッジ付き
uv run pytest -m "not render and not api" --cov   # CI と同じ(描画・実 API を除く)
uv run pytest tests/test_package.py   # 特定のテスト
```

アプリケーションの起動・ビルド・DB 操作は、現時点では該当なし。
