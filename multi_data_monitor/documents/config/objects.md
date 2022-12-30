# config node object

## Common objects

- [ConfigFile](common/config-file.md)
- [Stylesheet](common/stylesheet.md)
- [Widgets](common/widget-like.md) (WidgetLike / WidgetNode / WidgetRelay)
- [Streams](common/stream-like.md) (StreamLike / StreamNode / StreamRelay)
- [Filters](common/filter-like.md) (FilterLike, FilterNode, FilterRelay)

## Widget classes

- Simple
- Titled
- Matrix

## Stream classes

- subscription
- apply
- print
- @topic
- @field
- @panel

## Filter classes

- function
- SetIf
- SetFirstIf
- Units
- Precision

## Filter components

フィルタークラスの設定にて共通で利用するプロパティーの集合です。

- Conditions
- ValueAttrs

## Subscription objects

コンフィグファイル直下の `subscriptions` セクションで使用するオブジェクトです。

- SubscriptionTopic
- SubscriptionField
