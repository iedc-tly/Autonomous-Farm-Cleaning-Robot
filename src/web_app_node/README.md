# web_app_node

ROS2 bridge node: exposes HTTP API for web/Android app, publishes geometry_msgs/Twist on /cmd_vel.

## Install into your workspace

1. Extract this folder into `~/ros2_ws/src/web_app_node`
2. Replace `web_app_node/web_app_node/static/index.html` with your real UI
3. Install flask: `pip install flask`
4. Build:
   ```
   cd ~/ros2_ws
   colcon build --packages-select web_app_node
   source install/setup.bash
   ```
5. Run:
   ```
   ros2 run web_app_node app_node
   ```
6. Get RPi IP: `hostname -I`
7. Open `http://<rpi-ip>:8080` in browser

## API

- `GET /` → serves static/index.html
- `GET /api/ping` → health check
- `POST /api/drive` → JSON `{"mode": "forward|backward|left|right|stop", "speed": 0..1}`
- `POST /api/stop` → force stop

## Verify

```
ros2 topic echo /cmd_vel
```
Press a control button in the browser, confirm Twist messages appear.
