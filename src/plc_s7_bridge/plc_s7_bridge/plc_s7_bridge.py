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
    """Connect ROS 2 to DB bits on a Siemens S7-1214C PLC.

    Default mapping uses the PLC team's existing DB5 byte 8:
      DB5.DBX8.0: ROS requests robot power-on
      DB5.DBX8.1: ROS requests robot enable
      DB5.DBX8.2: ROS requests suction on, reserved for later
      DB5.DBX8.3: PLC reports power-on feedback ready
      DB5.DBX8.4: PLC reports enable feedback ready
      DB5.DBX8.5: PLC reports emergency stop triggered

    The DB number, byte offset, and bit offsets are parameters so the same node
    can be reused if the PLC program changes later.
    """

    def __init__(self) -> None:
        super().__init__("plc_s7_bridge")

        self.declare_parameter("plc_ip", "192.168.6.1")
        self.declare_parameter("rack", 0)
        self.declare_parameter("slot", 1)
        self.declare_parameter("db_number", 5)
        self.declare_parameter("io_byte", 8)
        self.declare_parameter("power_request_bit", 0)
        self.declare_parameter("enable_request_bit", 1)
        self.declare_parameter("suction_request_bit", 2)
        self.declare_parameter("power_ready_bit", 3)
        self.declare_parameter("enable_ready_bit", 4)
        self.declare_parameter("estop_bit", 5)
        self.declare_parameter("poll_period", 0.5)
        self.declare_parameter("auto_reconnect", True)

        self.plc_ip = str(self.get_parameter("plc_ip").value)
        self.rack = int(self.get_parameter("rack").value)
        self.slot = int(self.get_parameter("slot").value)
        self.db_number = int(self.get_parameter("db_number").value)
        self.io_byte = int(self.get_parameter("io_byte").value)
        self.power_request_bit = int(self.get_parameter("power_request_bit").value)
        self.enable_request_bit = int(self.get_parameter("enable_request_bit").value)
        self.suction_request_bit = int(self.get_parameter("suction_request_bit").value)
        self.power_ready_bit = int(self.get_parameter("power_ready_bit").value)
        self.enable_ready_bit = int(self.get_parameter("enable_ready_bit").value)
        self.estop_bit = int(self.get_parameter("estop_bit").value)
        self.auto_reconnect = bool(self.get_parameter("auto_reconnect").value)

        self._plc = snap7.client.Client()
        self._plc_lock = threading.RLock()

        self.robot_ready: Optional[bool] = None
        self.e_stop: Optional[bool] = None
        self.suction_feedback: Optional[bool] = None
        self.object_detected: Optional[bool] = None
        self.plc_alarm: Optional[bool] = None
        self.power_ready: Optional[bool] = None
        self.enable_ready: Optional[bool] = None

        self.power_on_cmd = False
        self.enable_cmd = False
        self.suction_on_cmd = False
        self.suction_release_cmd = False
        self.reset_cmd = False

        self.ready_pub = self.create_publisher(Bool, "plc/robot_ready", 10)
        self.power_ready_pub = self.create_publisher(Bool, "plc/power_ready", 10)
        self.enable_ready_pub = self.create_publisher(Bool, "plc/enable_ready", 10)
        self.estop_pub = self.create_publisher(Bool, "plc/e_stop", 10)
        self.suction_feedback_pub = self.create_publisher(
            Bool, "plc/suction_on_feedback", 10
        )
        self.object_detected_pub = self.create_publisher(
            Bool, "plc/object_detected", 10
        )
        self.plc_alarm_pub = self.create_publisher(Bool, "plc/plc_alarm", 10)

        self.create_service(SetBool, "plc/set_power_on", self._handle_set_power_on)
        self.create_service(SetBool, "plc/set_enable", self._handle_set_enable)
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
                        f"成功连接到 PLC: {self.plc_ip}, rack={self.rack}, slot={self.slot}, "
                        f"DB{self.db_number}.DBX{self.io_byte}.0~7"
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
        """Read one PLC I/O byte from the configured DB byte offset."""
        try:
            with self._plc_lock:
                data = self._plc.db_read(self.db_number, self.io_byte, 1)

            power_ready = bool(get_bool(data, 0, self.power_ready_bit))
            enable_ready = bool(get_bool(data, 0, self.enable_ready_bit))
            estop_status = bool(get_bool(data, 0, self.estop_bit))
            ready_status = power_ready and enable_ready and not estop_status

            # 当前阶段吸盘未接气泵，下面三个 topic 先保持 False，后续接气路再扩展。
            suction_feedback = False
            object_detected = False
            plc_alarm = False

            if (
                ready_status != self.robot_ready
                or power_ready != self.power_ready
                or enable_ready != self.enable_ready
                or estop_status != self.e_stop
                or suction_feedback != self.suction_feedback
                or object_detected != self.object_detected
                or plc_alarm != self.plc_alarm
            ):
                self.robot_ready = ready_status
                self.power_ready = power_ready
                self.enable_ready = enable_ready
                self.e_stop = estop_status
                self.suction_feedback = suction_feedback
                self.object_detected = object_detected
                self.plc_alarm = plc_alarm
                self.get_logger().info(
                    "PLC 状态更新: "
                    f"上电反馈已就绪={self.power_ready}, "
                    f"使能反馈已就绪={self.enable_ready}, "
                    f"机械臂就绪={self.robot_ready}, "
                    f"急停={self.e_stop}, "
                    f"吸盘输出反馈={self.suction_feedback}, "
                    f"吸住工件={self.object_detected}, "
                    f"PLC报警={self.plc_alarm}"
                )

            self.ready_pub.publish(Bool(data=ready_status))
            self.power_ready_pub.publish(Bool(data=power_ready))
            self.enable_ready_pub.publish(Bool(data=enable_ready))
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
        enable: bool,
        suction_on: bool,
        suction_release: bool,
        reset: bool,
    ) -> bool:
        """Write command bits to the configured PLC I/O byte."""
        if not self._ensure_connected():
            self.get_logger().error("PLC 未连接，无法写入指令。")
            return False

        try:
            with self._plc_lock:
                data = self._plc.db_read(self.db_number, self.io_byte, 1)
                set_bool(data, 0, self.power_request_bit, power_on)
                set_bool(data, 0, self.enable_request_bit, enable)
                set_bool(data, 0, self.suction_request_bit, suction_on)
                self._plc.db_write(self.db_number, self.io_byte, data)

            self.power_on_cmd = power_on
            self.enable_cmd = enable
            self.suction_on_cmd = suction_on
            self.suction_release_cmd = suction_release
            self.reset_cmd = reset
            self.get_logger().info(
                "PLC 指令下发: "
                f"请求上电={power_on}, "
                f"请求使能={enable}, "
                f"吸盘吸气={suction_on}, "
                f"破真空/放气={suction_release}, "
                f"复位={reset}, "
                f"写入地址=DB{self.db_number}.DBB{self.io_byte}"
            )
            return True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"写入 PLC 指令失败: {exc}")
            self._mark_disconnected()
            return False

    def _handle_set_power_on(self, request: SetBool.Request, response: SetBool.Response):
        ok = self._write_plc_command(
            request.data,
            self.enable_cmd,
            self.suction_on_cmd,
            self.suction_release_cmd,
            self.reset_cmd,
        )
        response.success = ok
        response.message = "power_on command written" if ok else "failed to write power_on command"
        return response

    def _handle_set_enable(self, request: SetBool.Request, response: SetBool.Response):
        ok = self._write_plc_command(
            self.power_on_cmd,
            request.data,
            self.suction_on_cmd,
            self.suction_release_cmd,
            self.reset_cmd,
        )
        response.success = ok
        response.message = "enable command written" if ok else "failed to write enable command"
        return response

    def _handle_set_suction_on(self, request: SetBool.Request, response: SetBool.Response):
        ok = self._write_plc_command(
            self.power_on_cmd,
            self.enable_cmd,
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
            self.enable_cmd,
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
            self.enable_cmd,
            self.suction_on_cmd,
            self.suction_release_cmd,
            request.data,
        )
        response.success = ok
        response.message = "reset command written" if ok else "failed to write reset command"
        return response

    def _handle_clear_commands(self, request: Trigger.Request, response: Trigger.Response):
        del request
        ok = self._write_plc_command(False, False, False, False, False)
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
