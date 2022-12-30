# ConfigFile

## Description

コンフィグファイルを構成する最上位のオブジェクトです。コンフィグファイル全体は辞書型のデータである必要があります。

## Format

| Key Name      | Value Type                                                   | Description                                                     |
| ------------- | ------------------------------------------------------------ | --------------------------------------------------------------- |
| version       | string                                                       | コンフィグファイルのバージョン。現在は 2.0 のみ対応しています。 |
| stylesheets   | List of [Stylesheet](../common/stylesheet.md)                | 使用する stylesheet を指定します。                              |
| widgets       | List of [WidgetLike](../common/widget-like.md)               | 使用する widget を定義します。                                  |
| streams       | List of [StreamLike](../common/stream-like.md)               | 使用する stream を定義します。                                  |
| filters       | List of [FilterLike](../common/filter-like.md)               | 使用する filter を定義します。                                  |
| subscriptions | List of [SubscriptionTopic](../stream/subscription-topic.md) | 使用する subscription を定義します。                            |
