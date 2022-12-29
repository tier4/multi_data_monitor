# コンフィグファイル処理手順

## Step 1. load yaml file

Type: `Path => ConfigFile`

ファイルの読み込みやバージョンチェックなどを行う。

## Step 2. construct node

Type: `ConfigFile => ConfigData`

木構造を解析して各ノードの構築を行う。

## Step 3. tramsform node

Type: `ConfigData => ConfigData`

特殊挙動する組み込みクラスを変換する。

## Step 4. resolve link

Type: `ConfigData => ConfigData`

TODO: 循環参照検出
