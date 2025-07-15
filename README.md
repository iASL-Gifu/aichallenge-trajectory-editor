# aichallenge-trajectory-editor
This project is a plugin for editing vehicle tracks on Rviz.
# AIChallenge Trajectory Editorの使い方
aichallenge trajectory editorの使い方
まず、このレポジトリを任意のwsにcloneしてください。
今回は,`~/aichallenge/workspace/src`配下にcloneすることをおすすめします。
```bash
git clone https://github.com/iASL-Gifu/aichallenge-trajectory-editor.git
```

次に`~/aichallenge-2025/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/aichallenge_submit.launch.xml`に以下を追加します。

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
次にdockerに入り、次のコマンドを実行します。
```bash
./build_autoware.bash
```

次にAutowareを起動し、Rviz画面上部の`Pannel->Add new panel`を選択します。

![image](./asset/panel.png)

次に`rviz_editor_plugins`にあるCsvMarkerDisplay・EditorToolを追加します。

![image2](./asset/newpanel.png)

新たにdisplayが追加されます。

![rviz2](./asset/rviz2.png)

最後に、`/race_trajectory`と`/editor_tool_server`のtopicを描画して下さい。
![topics](./asset/topics.png)

以下のように表示されます。必要に応じてRviz画面をFile→Saveして下さい。
![rviz2](./asset/rviz.png)



# 機能説明

load csv: csvを指定することでtrajectoryをloadできます。

save csv: trajectoryをsaveする機能です。　

select range: 上部のtext editorに埋め込まれている速度を指定した区間に反映します。　

start parallel move & end parallel move: 選択した２点間にあるすべての点を平行移動できるようにします。終了時にendしてください。

post trajectory: Editorの車両経路をAutowareに反映させます。

