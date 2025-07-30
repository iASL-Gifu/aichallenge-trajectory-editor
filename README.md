# aichallenge-trajectory-editor
This project is a plugin for editing vehicle tracks on Rviz.
  ![rviz](./asset/rviz.png)

## 主な機能
`aichallenge-trajectory-editor`は、ROS 2 および Autoware 環境向けに開発された、車両の軌跡を編集するためのRvizプラグインです。このリポジトリは、以下の主要な機能を提供しています。

* **軌跡データの管理**
    * CSV形式の軌跡データを読み込み、Rviz上に表示することができます。
    * 編集した軌跡データをCSVファイルとして保存することができます。
* **Rviz上での軌跡編集**
    * **速度の変更**: Rviz上で軌跡上の任意の点(複数点も可能)を選択し、その軌跡ポイントの速度を変更できます。速度に応じてマーカーの色が自動的に変更されます。
    * **平行移動**: 選択した2点間のすべての点を平行移動させることができます。
    * **Undo/Redo**: 軌跡編集の操作を元に戻したり、やり直したりする機能があります。
* **Autowareとの連携**
    * 編集した軌跡は、Autowareの`/planning/scenario_planning/trajectory`トピックにパブリッシュすることが可能です。これにより、編集結果をAutowareのプランニングモジュールに反映させることができます。
    * 軌跡の初期読み込みやパブリッシュに関するパラメータ（例：`csv_file_path`、`publish_on_initialize`、`wait_seconds`、`grad_min_speed`、`grad_mid_speed`、`grad_max_speed`）を設定ファイルで定義できます。

このツールは、Autowareで自動運転シミュレーションを行う際に、車両が走行する軌跡を手動で調整し、その影響を即座に確認できるインタラクティブな環境を提供することを目的としています。
はい、承知いたしました。提供された情報を基に、`AIChallenge Trajectory Editor`のセットアップと使用方法について、より整理して記述します。

-----

## セットアップと使用方法

`AIChallenge Trajectory Editor`は、Autoware環境で車両の軌跡をRviz上で視覚的に編集するためのツールです。以下の手順でセットアップと使用が可能です。

### 1\. リポジトリのクローン

まず、任意のワークスペース（例: `~/aichallenge/workspace/src/aichallenge_submit/`）に本リポジトリをクローンします。

```bash
cd ~/aichallenge-2025/aichallenge/workspace/src/aichallenge_submit
git clone https://github.com/iASL-Gifu/aichallenge-trajectory-editor.git
```

### 2\. Autoware起動設定の追加

Autowareの起動設定ファイル`~/aichallenge-2025/aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/aichallenge_submit.launch.xml`に、以下の`editor_tool_server`ノードを追加します。

```xml
<node pkg="editor_tool_server" exec="interactive_server" output="screen" name="editor_tool_server">
    <param name="csv_file_path" value="/aichallenge/workspace/src/aichallenge-trajectory-editor/csv/centerline_15km.csv"/>
    <param name="publish_on_initialize" value="true"/>
    <param name="wait_seconds" value="5.0"/>
    <param name="grad_min_speed" value="0.0"/>
    <param name="grad_mid_speed" value="20.0"/>
    <param name="grad_max_speed" value="40.0"/>
</node>
```

この設定では、`editor_tool_server`ノードが起動時にデフォルトのCSVファイルを読み込み、初期軌跡をパブリッシュするようになっています。

### 3\. Autowareのビルドと起動

Docker環境に入り、以下のコマンドでAutowareをビルドします。

```bash
./build_autoware.bash
```

ビルドが完了したら、Autowareを起動します。

### 4\. Rvizプラグインの追加
本リポジトリには、Rvizの設定ファイル autoware.rviz が含まれています。このファイルをRvizのFile -> Open Configから読み込むことで、必要なプラグインやトピック表示設定を簡単に適用できます。

手動で設定する場合は、Autoware起動後、Rviz画面を開き、画面上部のメニューから`Panel` -\> `Add New Panel`を選択します。

表示されるウィンドウで、`rviz_editor_plugins`カテゴリにある`CsvMarkerDisplay`と`EditorTool`をそれぞれ追加します。

![newpanel](./asset/newpanel.png)

### 5\. トピックの描画設定

Rviz画面の左下にある`Displays`パネルで、以下のトピックを追加して描画します。

  * `/race_trajectory`
  * `/editor_tool_server`

![topics](./asset/topics.png)


これにより、編集対象の軌跡と、それを操作するためのインタラクティブマーカーが表示されます。必要に応じて、現在のRvizの設定を保存しておくことを推奨します (`File` -\> `Save`)。

### 6\. エディタの使用方法

Rvizに追加された`EditorTool`パネルと`CsvMarkerDisplay`パネルを使用して、軌跡を編集します。

  * **Load CSV (CsvMarkerDisplayパネル)**: 新しいCSVファイルをロードして、表示される軌跡を切り替えます。
  * **Save CSV (CsvMarkerDisplayパネル)**: 編集した軌跡データをCSVファイルとして保存します。
  * **Velocity入力 (EditorToolパネル)**: 速度を入力するテキストボックスです。ここに数値を入力し、「Select Range」ボタンと組み合わせて使用します。
  * **Select Range (EditorToolパネル)**: Velocity入力欄に入力された速度を、Rviz上でクリックして選択した2点間の軌跡に適用します。
  * **Start Parallel Move (EditorToolパネル)**: 軌跡上の2点を選択すると、青い球体のマーカーが生成され、その区間内のすべての軌跡ポイントを平行移動できるようになります。
  * **End Parallel Move (EditorToolパネル)**: 平行移動モードを終了し、軌跡の位置の変更を確定・保存します。
  * **Undo (EditorToolパネル)**: 直前の編集操作を元に戻します。
  * **Redo (EditorToolパネル)**: Undoで元に戻した操作をやり直します。
  * **Post Trajectory (EditorToolパネル)**: **RViz上で編集した経路をAutowareに反映させるためには、この「Post Trajectory」ボタンをクリックする必要があります。これを実行しない限り、RViz上での変更はAutowareのプランニングに適用されません。**
