# omni3_control
設計製作特論用
ラズパイ5でROS2を使っ三輪オムニを制御するノード
## install
```
cd ~/ros2_ws/src
git clone https://github.com/kokiikeda6/omni3_control.git
cd ~/ros2_ws
colcon build
```

## execute
1. 三輪オムニ起動
```
ros2 launch omni3_control bb300.launch.py
```
