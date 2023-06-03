# Filter

## Description

Filter はフィルター全般を表すオブジェクトです。指定したデータの型に応じてそれぞれ以下のように解釈されます。

| Type   | Description                                                                                                   |
| ------ | ------------------------------------------------------------------------------------------------------------- |
| dict   | プラグインの定義として解釈されます。以下に示すフォーマットのセクションを参照してください。                    |
| list   | プラグインの逐次実行として解釈されます。詳細については [function](../filter/function.md) を参照してください。 |
| string | フィルターへの参照として解釈されます。詳細については [Reference](./reference.md) を参照してください。         |

## Format

| Name  | Type   | Required | Description                                            |
| ----- | ------ | -------- | ------------------------------------------------------ |
| class | string | yes      | フィルターのプラグイン名を指定します。                 |
| label | string | no       | フィルターの参照に使用する任意の名称を指定します。     |
| -     | -      | -        | 他のフィールドは各プラグインの説明を参照してください。 |