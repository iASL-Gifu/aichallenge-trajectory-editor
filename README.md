# aichallenge-trajectory-editor
This project is a plugin for editing vehicle tracks on Rviz.
## How to build
```bash
mkdir ~/ws/src -p
cd ~/ws/src
git clone git@github.com:iASL-Gifu/aichallenge2024-trajectory-editor.git

cd ~/ws
colcon build --symlink-install
```

## How to run
```bash
# terminal 1
source install/setup.bash
rviz2 -d path/to/tools.rviz

# terminal 2
source install/setup.bash
ros2 run editor_tool_sever interactive_server
```

## How to use
![editor](./asset/rviz2_editor_tool.png)
1. load csv file button and select csv file.
2. edit trajectory
3. save & publish


[AIChallenge](./aichallenge_jp.md)

### select range:
Select two points and apply the number (speed) on the button to all trajectory between them.
### start parallel move:
Selecting two points will generate blue spherical markers. By moving these markers, you can parallel trajectories in the specified section.
### End parallel move:
Finish parallel move & position save

### other version
[trajectory editor(python)](https://github.com/iASL-Gifu/aichallenge-trajectory-editor/tree/trajectory-editor-python)
