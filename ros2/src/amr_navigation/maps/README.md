# Maps

Save maps here (`my_map.yaml` + `my_map.pgm`):

```bash
ros2 launch amr_navigation slam.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run nav2_map_server map_saver_cli -f ros2/src/amr_navigation/maps/my_map
```
