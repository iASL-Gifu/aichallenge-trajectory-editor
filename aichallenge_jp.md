# AIChallenge Trajectory Editorの使い方
aichallenge trajectory editorの使い方
まず、このレポジトリを任意のwsにcloneしてください。
今回は,`/aichallenge/workspace/src`配下にcloneすることをおすすめします。
```bash
git clone https://github.com/iASL-Gifu/aichallenge-trajectory-editor.git
```

次に`/aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/aichallenge_submit.launch.xml`のplanningを変更します。

```xml
<node pkg="editor_tool_server" exec="interactive_server" output="screen" name="editor_tool_server">
    <param name="csv_file_path" value="/aichallenge/workspace/src/aichallenge-trajectory-editor/csv/centerline_15km.csv"/>
    <param name="publish_on_initialize" value="true"/>
    <param name="wait_seconds" value="5.0"/>
    <param name="grad_min_speed" value="0.0"/>
    <param name="grad_mid_speed" value="30.0"/>
    <param name="grad_max_speed" value="60.0"/>
</node>
```
次にdockerに入り、次のコマンドを実行します。
```bash
./build_autoware.bash
```

おそらくエラーなく動作すると思います。それでは,autowareを起動しましょう。そうしたら、画面上部の`Pannel->Add new panel`を選択します。

![image](./asset/panel.png)

次に`rviz_editor_plugins`にある２つを起動します。

![image2](./asset/newpanel.png)

すると新たにdisplayが追加されているはずです。

![rviz2](./asset/rviz2.png)

最後に、`race_trajectory`と`editor_tool_server`のtopicを描画できるように選択しておいてください。ここからは各機能について説明します。

load csv: csvを指定することでtrajectoryをloadできます。

save csv: trajectoryをsaveする機能ですが,実装が遅れております。　

select range: 上部のtext editorに埋め込まれている速度を指定した区間に反映します。　

start parallel move & end parallel move: 選択した２点間にあるすべての点を平行移動できるようにします。終了時にendしてください。

post trajectory: 実際の車両起動をeditorのものに反映します。
