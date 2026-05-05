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
    """Connect ROS 2 to DB1 bits on a Siemens S7-1214C PLC.

    DB1 command byte, ROS -> PLC:
      DBX0.0: robot power/enable request
      DBX0.1: suction on request
      DBX0.2: suction release/blow-off request
      DBX0.3: alarm reset request

    DB1 status byte, PLC -> ROS:
      DBX1.0: robot ready
      DBX1.1: emergency stop active
      DBX1.2: suction output feedback
      DBX1.3: object detected by vacuum switch
      DBX1.4: PLC/air circuit alarm
    """

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
        self.suction_feedback: Optional[bool] = None
        self.object_detected: Optional[bool] = None
        self.plc_alarm: Optional[bool] = None

        self.power_on_cmd = False
        self.suction_on_cmd = False
        self.suction_release_cmd = False
        self.reset_cmd = False

        self.ready_pub = self.create_publisher(Bool, "plc/robot_ready", 10)
        self.estop_pub = self.create_publisher(Bool, "plc/e_stop", 10)
        self.suction_feedback_pub = self.create_publisher(
            Bool, "plc/suction_on_feedback", 10
        )
        self.object_detected_pub = self.create_publisher(
            Bool, "plc/object_detected", 10
        )
        self.plc_alarm_pub = self.create_publisher(Bool, "plc/plc_alarm", 10)

        self.create_service(SetBool, "plc/set_power_on", self._handle_set_power_on)
        self.create_service(SetBool, "plc/set_suction_on", self._handle_set_suction_on)
        self.create_service(
            SetBool, "plc/set_suction_release", self._handle_set_suction_release
        )
        self.create_service(SetBool, "plc/set_reset", self._handle_set_reset)
        # 兼容早期命令名：现场没有夹爪时，把 gripper_close 当作 suction_on 使用。
        self.create_service(
            SetBool, "plc/set_gripper_close", self._handle_set_suction_on
        )
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
        """Read one PLC status byte from DB1 byte 1."""
        try:
            with self._plc_lock:
                data = self._plc.db_read(self.db_number, 1, 1)

            ready_status = bool(get_bool(data, 0, 0))
            estop_status = bool(get_bool(data, 0, 1))
            suction_feedback = bool(get_bool(data, 0, 2))
            object_detected = bool(get_bool(data, 0, 3))
            plc_alarm = bool(get_bool(data, 0, 4))

            if (
                ready_status != self.robot_ready
                or estop_status != self.e_stop
                or suction_feedback != self.suction_feedback
                or object_detected != self.object_detected
                or plc_alarm != self.plc_alarm
            ):
                self.robot_ready = ready_status
                self.e_stop = estop_status
                self.suction_feedback = suction_feedback
                self.object_detected = object_detected
                self.plc_alarm = plc_alarm
                self.get_logger().info(
                    "PLC 状态更新: "
                    f"机械臂就绪={self.robot_ready}, "
                    f"急停={self.e_stop}, "
                    f"吸盘输出反馈={self.suction_feedback}, "
                    f"吸住工件={self.object_detected}, "
                    f"PLC报警={self.plc_alarm}"
                )

            self.ready_pub.publish(Bool(data=ready_status))
            self.estop_pub.publish(Bool(data=estop_status))
            self.suction_feedback_pub.publish(Bool(data=suction_feedback))
            self.object_detected_pub.publish(Bool(data=object_detected))
            self.plc_alarm_pub.publish(Bool(data=plc_alarm))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"读取 PLC 状态失败: {exc}")
            self._mark_disconnected()

    def _write_plc_command(
        self,
        power_on: bool,
        suction_on: bool,
        suction_release: bool,
        reset: bool,
    ) -> bool:
        """Write one PLC command byte to DB1 byte 0."""
        if not self._ensure_connected():
            self.get_logger().error("PLC 未连接，无法写入指令。")
            return False

        try:
            with self._plc_lock:
                data = self._plc.db_read(self.db_number, 0, 1)
                set_bool(data, 0, 0, power_on)
                set_bool(data, 0, 1, suction_on)
                set_bool(data, 0, 2, suction_release)
                set_bool(data, 0, 3, reset)
                self._plc.db_write(self.db_number, 0, data)

            self.power_on_cmd = power_on
            self.suction_on_cmd = suction_on
            self.suction_release_cmd = suction_release
            self.reset_cmd = reset
            self.get_logger().info(
                "PLC 指令下发: "
                f"上电使能={power_on}, "
                f"吸盘吸气={suction_on}, "
                f"破真空/放气={suction_release}, "
                f"复位={reset}"
            )
            return True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"写入 PLC 指令失败: {exc}")
            self._mark_disconnected()
            return False

    def _handle_set_power_on(self, request: SetBool.Request, response: SetBool.Response):
        ok = self._write_plc_command(
            request.data,
            self.suction_on_cmd,
            self.suction_release_cmd,
            self.reset_cmd,
        )
        response.success = ok
        response.message = "power_on command written" if ok else "failed to write power_on command"
        return response

    def _handle_set_suction_on(self, request: SetBool.Request, response: SetBool.Response):
        ok = self._write_plc_command(
            self.power_on_cmd,
            request.data,
            self.suction_release_cmd,
            self.reset_cmd,
        )
        response.success = ok
        response.message = "suction_on command written" if ok else "failed to write suction_on command"
        return response

    def _handle_set_suction_release(
        self, request: SetBool.Request, response: SetBool.Response
    ):
        ok = self._write_plc_command(
            self.power_on_cmd,
            self.suction_on_cmd,
            request.data,
            self.reset_cmd,
        )
        response.success = ok
        response.message = (
            "suction_release command written"
            if ok
            else "failed to write suction_release command"
        )
        return response

    def _handle_set_reset(self, request: SetBool.Request, response: SetBool.Response):
        ok = self._write_plc_command(
            self.power_on_cmd,
            self.suction_on_cmd,
            self.suction_release_cmd,
            request.data,
        )
        response.success = ok
        response.message = "reset command written" if ok else "failed to write reset command"
        return response

    def _handle_clear_commands(self, request: Trigger.Request, response: Trigger.Response):
        del request
        ok = self._write_plc_command(False, False, False, False)
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
