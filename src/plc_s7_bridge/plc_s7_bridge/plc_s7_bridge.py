#!/usr/bin/env python3
"""ROS 2 bridge for Siemens S7-1214C PLC DB bit control."""

import threading
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

try:
    import snap7
    from snap7.util import get_bool, set_bool
except ImportError as exc:
    raise ImportError(
        "缺少 python-snap7。请先安装: python3 -m pip install python-snap7"
    ) from exc


class PlcS7Bridge(Node):
    """Connect ROS 2 to DB1 bits on a Siemens S7-1214C PLC."""

    def __init__(self) -> None:
        super().__init__("plc_s7_bridge")

        self.declare_parameter("plc_ip", "192.168.6.1")
        self.declare_parameter("rack", 0)
        self.declare_parameter("slot", 1)
        self.declare_parameter("db_number", 1)
        self.declare_parameter("poll_period", 0.5)
        self.declare_parameter("auto_reconnect", True)

        self.plc_ip = str(self.get_parameter("plc_ip").value)
        self.rack = int(self.get_parameter("rack").value)
        self.slot = int(self.get_parameter("slot").value)
        self.db_number = int(self.get_parameter("db_number").value)
        self.auto_reconnect = bool(self.get_parameter("auto_reconnect").value)

        self._plc = snap7.client.Client()
        self._plc_lock = threading.RLock()

        self.robot_ready: Optional[bool] = None
        self.e_stop: Optional[bool] = None
        self.power_on_cmd = False
        self.gripper_close_cmd = False

        self.ready_pub = self.create_publisher(Bool, "plc/robot_ready", 10)
        self.estop_pub = self.create_publisher(Bool, "plc/e_stop", 10)

        self.create_service(SetBool, "plc/set_power_on", self._handle_set_power_on)
        self.create_service(SetBool, "plc/set_gripper_close", self._handle_set_gripper_close)
        self.create_service(Trigger, "plc/clear_commands", self._handle_clear_commands)

        self._connect_to_plc()
        self.timer = self.create_timer(
            float(self.get_parameter("poll_period").value), self._timer_callback
        )

    def _connect_to_plc(self) -> bool:
        with self._plc_lock:
            try:
                if self._plc.get_connected():
                    return True
                self._plc.connect(self.plc_ip, self.rack, self.slot)
                if self._plc.get_connected():
                    self.get_logger().info(
                        f"成功连接到 PLC: {self.plc_ip}, rack={self.rack}, slot={self.slot}, DB{self.db_number}"
                    )
                    return True
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"PLC 连接失败: {exc}")
            return False

    def _timer_callback(self) -> None:
        if self._ensure_connected():
            self._read_plc_status()

    def _ensure_connected(self) -> bool:
        with self._plc_lock:
            if self._plc.get_connected():
                return True
        return self.auto_reconnect and self._connect_to_plc()

    def _read_plc_status(self) -> None:
        """Read DB1.DBX1.0 ready and DB1.DBX1.1 emergency stop status."""
        try:
            with self._plc_lock:
                data = self._plc.db_read(self.db_number, 1, 1)

            ready_status = bool(get_bool(data, 0, 0))
            estop_status = bool(get_bool(data, 0, 1))

            if ready_status != self.robot_ready or estop_status != self.e_stop:
                self.robot_ready = ready_status
                self.e_stop = estop_status
                self.get_logger().info(
                    f"PLC 状态更新: 机械臂就绪={self.robot_ready}, 急停={self.e_stop}"
                )

            self.ready_pub.publish(Bool(data=ready_status))
            self.estop_pub.publish(Bool(data=estop_status))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"读取 PLC 状态失败: {exc}")
            self._mark_disconnected()

    def _write_plc_command(self, power_on: bool, gripper_close: bool) -> bool:
        """Write DB1.DBX0.0 power-on and DB1.DBX0.1 gripper-close command bits."""
        if not self._ensure_connected():
            self.get_logger().error("PLC 未连接，无法写入指令。")
            return False

        try:
            with self._plc_lock:
                data = self._plc.db_read(self.db_number, 0, 1)
                set_bool(data, 0, 0, power_on)
                set_bool(data, 0, 1, gripper_close)
                self._plc.db_write(self.db_number, 0, data)

            self.power_on_cmd = power_on
            self.gripper_close_cmd = gripper_close
            self.get_logger().info(
                f"PLC 指令下发: 上电使能={power_on}, 夹爪闭合={gripper_close}"
            )
            return True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"写入 PLC 指令失败: {exc}")
            self._mark_disconnected()
            return False

    def _handle_set_power_on(self, request: SetBool.Request, response: SetBool.Response):
        ok = self._write_plc_command(request.data, self.gripper_close_cmd)
        response.success = ok
        response.message = "power_on command written" if ok else "failed to write power_on command"
        return response

    def _handle_set_gripper_close(self, request: SetBool.Request, response: SetBool.Response):
        ok = self._write_plc_command(self.power_on_cmd, request.data)
        response.success = ok
        response.message = "gripper_close command written" if ok else "failed to write gripper_close command"
        return response

    def _handle_clear_commands(self, request: Trigger.Request, response: Trigger.Response):
        del request
        ok = self._write_plc_command(False, False)
        response.success = ok
        response.message = "commands cleared" if ok else "failed to clear commands"
        return response

    def _mark_disconnected(self) -> None:
        with self._plc_lock:
            try:
                self._plc.disconnect()
            except Exception:  # noqa: BLE001
                pass

    def destroy_node(self) -> bool:
        with self._plc_lock:
            try:
                if self._plc.get_connected():
                    self._plc.disconnect()
            except Exception:  # noqa: BLE001
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlcS7Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C，正在退出 plc_s7_bridge。")
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
