# コマンドラインツール

## Config parser

コンフィグファイルの構造を確認するコマンドです。結果は plantuml 形式で出力されるので各自でレンダリングしてください。

```bash
ros2 run multi_data_monitor parser <scheme> <path>
```

| 引数   | 説明                                                |
| ------ | --------------------------------------------------- |
| scheme | Rvizでの利用と同じく file か package を指定します。 |
| path   | 上記の scheme に対応したパスを指定します。          |

## Config runner

Rvizを使用せずにデータの流れのみを確認するコマンドです。現在作成中。
