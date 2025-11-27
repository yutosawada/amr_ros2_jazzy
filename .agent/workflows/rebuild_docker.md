---
description: Docker環境をリセットして再ビルドし、起動する
---

1. コンテナとボリュームを停止・削除します
// turbo
docker compose down -v

2. キャッシュなしで再ビルドします
// turbo
docker compose build --no-cache

3. コンテナを起動します
// turbo
docker compose up -d
