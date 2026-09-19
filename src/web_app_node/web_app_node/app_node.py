#!/usr/bin/env python3
import asyncio
import json
import logging
import os
import threading
from typing import Any, Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, String

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("agri_rover_websocket_node")

HOST = os.getenv("ROVER_WS_HOST", "0.0.0.0")
PORT = int(os.getenv("ROVER_WS_PORT", "8000"))

app = FastAPI(title="Agri Rover WebSocket Node", version="1.0.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class RoverState:
    def __init__(self) -> None:
        self.battery = 82
        self.signal = "Strong"
        self.operation_time = "1h 15m"
        self.zone = "Sector 4-B"
        self.diagnostic_ok = True
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.speed_scale = 0.6
        self.e_stop = False
        self.auto_clean_active = False
        self.auto_clean_cycles = 3

    def telemetry(self) -> dict[str, Any]:
        return {
            "battery": self.battery,
            "signal": self.signal,
            "operation_time": self.operation_time,
            "zone": self.zone,
            "diagnostic_ok": self.diagnostic_ok,
        }


STATE = RoverState()
active_connections: set[WebSocket] = set()
ros_node: Optional["AppNode"] = None
main_event_loop: Optional[asyncio.AbstractEventLoop] = None


class AppNode(Node):
    def __init__(self, state: RoverState):
        super().__init__('web_app_node')
        self.state = state

        # ROS 2 Publishers (Flutter App -> ROS 2 Topics)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.estop_pub = self.create_publisher(Bool, '/rover/e_stop', 10)
        self.speed_scale_pub = self.create_publisher(Float32, '/rover/speed_scale', 10)
        self.auto_clean_pub = self.create_publisher(String, '/rover/auto_clean', 10)

        # ROS 2 Subscribers (ROS 2 Topics -> Flutter App)
        self.telemetry_sub = self.create_subscription(
            String, '/rover/telemetry', self.telemetry_cb, 10
        )

        self.get_logger().info('Agri Rover ROS 2 Bridge Node initialized')

    def publish_cmd_vel(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def publish_estop(self, e_stop: bool):
        msg = Bool()
        msg.data = bool(e_stop)
        self.estop_pub.publish(msg)

    def publish_speed_scale(self, scale: float):
        msg = Float32()
        msg.data = float(scale)
        self.speed_scale_pub.publish(msg)

    def publish_auto_clean(self, active: bool, cycles: int):
        msg = String()
        msg.data = json.dumps({"active": bool(active), "cycles": int(cycles)})
        self.auto_clean_pub.publish(msg)

    def telemetry_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                self.state.battery = data.get("battery", self.state.battery)
                self.state.signal = data.get("signal", self.state.signal)
                self.state.operation_time = data.get("operation_time", self.state.operation_time)
                self.state.zone = data.get("zone", self.state.zone)
                self.state.diagnostic_ok = data.get("diagnostic_ok", self.state.diagnostic_ok)

                if main_event_loop and main_event_loop.is_running():
                    asyncio.run_coroutine_threadsafe(broadcast_telemetry(), main_event_loop)
        except Exception as e:
            self.get_logger().error(f"Error parsing telemetry msg: {e}")


def _publish_telemetry() -> str:
    return json.dumps(
        {
            "topic": "/rover/telemetry",
            "msg": json.dumps(STATE.telemetry()),
        }
    )


def _publish_status() -> str:
    return json.dumps(
        {
            "topic": "/rover/status",
            "msg": json.dumps(
                {
                    "connected": True,
                    "e_stop": STATE.e_stop,
                    "auto_clean_active": STATE.auto_clean_active,
                    "cycles": STATE.auto_clean_cycles,
                    "speed_scale": STATE.speed_scale,
                }
            ),
        }
    )


async def broadcast_status() -> None:
    status_text = _publish_status()
    for connection in list(active_connections):
        try:
            await connection.send_text(status_text)
        except Exception:
            pass


async def broadcast_telemetry() -> None:
    telemetry_text = _publish_telemetry()
    for connection in list(active_connections):
        try:
            await connection.send_text(telemetry_text)
        except Exception:
            pass


@app.get("/health")
async def health() -> dict[str, object]:
    return {
        "status": "ok",
        "host": HOST,
        "port": PORT,
        "ros2_connected": ros_node is not None,
        "note": "Agri Rover ROS2 WebSocket Node connected.",
    }


@app.websocket("/ws/rover")
async def rover_node(client: WebSocket) -> None:
    await client.accept()
    active_connections.add(client)
    logger.info("Flutter client connected to /ws/rover")

    await client.send_text(
        json.dumps(
            {
                "op": "status",
                "connected": True,
                "ws": "/ws/rover",
                "mode": "ros2",
            }
        )
    )
    await client.send_text(_publish_status())
    await client.send_text(_publish_telemetry())

    try:
        while True:
            raw = await client.receive_text()
            try:
                payload = json.loads(raw)
            except json.JSONDecodeError:
                await client.send_text(
                    json.dumps(
                        {
                            "op": "error",
                            "message": "Invalid JSON received",
                        }
                    )
                )
                continue

            op = payload.get("op")
            topic = payload.get("topic")
            msg = payload.get("msg", {})

            if op == "publish":
                if topic == "/cmd_vel":
                    linear = msg.get("linear", {}) if isinstance(msg, dict) else {}
                    angular = msg.get("angular", {}) if isinstance(msg, dict) else {}
                    STATE.linear_x = float(linear.get("x", 0.0))
                    STATE.angular_z = float(angular.get("z", 0.0))
                    STATE.diagnostic_ok = True

                    if ros_node:
                        ros_node.publish_cmd_vel(STATE.linear_x, STATE.angular_z)

                    await broadcast_status()
                elif topic == "/rover/e_stop":
                    STATE.e_stop = bool(msg.get("data", False)) if isinstance(msg, dict) else False
                    if STATE.e_stop:
                        STATE.linear_x = 0.0
                        STATE.angular_z = 0.0
                        if ros_node:
                            ros_node.publish_cmd_vel(0.0, 0.0)

                    if ros_node:
                        ros_node.publish_estop(STATE.e_stop)

                    await broadcast_status()
                elif topic == "/rover/speed_scale":
                    STATE.speed_scale = float(msg.get("data", STATE.speed_scale)) if isinstance(msg, dict) else STATE.speed_scale

                    if ros_node:
                        ros_node.publish_speed_scale(STATE.speed_scale)

                    await broadcast_status()
                elif topic == "/rover/auto_clean":
                    if isinstance(msg, dict):
                        STATE.auto_clean_active = bool(msg.get("active", False))
                        STATE.auto_clean_cycles = int(msg.get("cycles", STATE.auto_clean_cycles))
                        if ros_node:
                            ros_node.publish_auto_clean(STATE.auto_clean_active, STATE.auto_clean_cycles)

                    await broadcast_status()

                await broadcast_telemetry()
                continue

            if op == "subscribe":
                await client.send_text(_publish_status())
                await client.send_text(_publish_telemetry())
                continue

            if op == "call_service":
                await client.send_text(
                    json.dumps(
                        {
                            "op": "service_response",
                            "service": payload.get("service", "unknown"),
                            "values": {"success": True},
                        }
                    )
                )
                continue

            await client.send_text(
                json.dumps(
                    {
                        "op": "error",
                        "message": f"Unsupported op: {op}",
                    }
                )
            )
    except WebSocketDisconnect:
        logger.info("Flutter client disconnected")
    finally:
        active_connections.discard(client)


def main():
    global ros_node, main_event_loop
    rclpy.init()

    ros_node = AppNode(STATE)

    def spin_ros():
        try:
            rclpy.spin(ros_node)
        except Exception as e:
            logger.info(f"ROS 2 spin finished: {e}")

    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()

    @app.on_event("startup")
    async def startup_event():
        global main_event_loop
        main_event_loop = asyncio.get_running_loop()

    logger.info(f"Starting Agri Rover WebSocket Node on http://{HOST}:{PORT}")
    try:
        uvicorn.run(app, host=HOST, port=PORT, log_level="info")
    finally:
        if ros_node:
            ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
