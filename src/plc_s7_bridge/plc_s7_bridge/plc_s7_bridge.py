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

    Default mapping uses the PLC team's existing DB5 byte 8/9:
      DB5.DBX8.0: ROS requests robot power-on
      DB5.DBX8.1: ROS requests robot power-off
      DB5.DBX8.2: ROS requests robot enable
      DB5.DBX8.3: ROS requests robot disable
      DB5.DBX8.4: ROS requests suction on, reserved for later
      DB5.DBX8.5: PLC reports power-on feedback ready
      DB5.DBX8.6: PLC reports enable feedback ready
      DB5.DBX8.7: PLC reports power-off feedback ready
      DB5.DBX9.0: PLC reports disable feedback ready

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
        self.declare_parameter("power_on_request_bit", 0)
        self.declare_parameter("power_off_request_bit", 1)
        self.declare_parameter("enable_request_bit", 2)
        self.declare_parameter("disable_request_bit", 3)
        self.declare_parameter("suction_request_bit", 4)
        self.declare_parameter("power_on_ready_bit", 5)
        self.declare_parameter("enable_ready_bit", 6)
        self.declare_parameter("power_off_ready_bit", 7)
        self.declare_parameter("disable_ready_byte", 9)
        self.declare_parameter("disable_ready_bit", 0)
        self.declare_parameter("estop_byte", -1)
        self.declare_parameter("estop_bit", -1)
        self.declare_parameter("poll_period", 0.5)
        self.declare_parameter("auto_reconnect", True)

        self.plc_ip = str(self.get_parameter("plc_ip").value)
        self.rack = int(self.get_parameter("rack").value)
        self.slot = int(self.get_parameter("slot").value)
        self.db_number = int(self.get_parameter("db_number").value)
        self.io_byte = int(self.get_parameter("io_byte").value)
        self.power_on_request_bit = int(self.get_parameter("power_on_request_bit").value)
        self.power_off_request_bit = int(self.get_parameter("power_off_request_bit").value)
        self.enable_request_bit = int(self.get_parameter("enable_request_bit").value)
        self.disable_request_bit = int(self.get_parameter("disable_request_bit").value)
        self.suction_request_bit = int(self.get_parameter("suction_request_bit").value)
        self.power_on_ready_bit = int(self.get_parameter("power_on_ready_bit").value)
        self.enable_ready_bit = int(self.get_parameter("enable_ready_bit").value)
        self.power_off_ready_bit = int(self.get_parameter("power_off_ready_bit").value)
        self.disable_ready_byte = int(self.get_parameter("disable_ready_byte").value)
        self.disable_ready_bit = int(self.get_parameter("disable_ready_bit").value)
        self.estop_byte = int(self.get_parameter("estop_byte").value)
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
        self.power_off_ready: Optional[bool] = None
        self.disable_ready: Optional[bool] = None

        self.power_on_request = False
        self.power_off_request = False
        self.enable_request = False
        self.disable_request = False
        self.suction_on_cmd = False
        self.suction_release_cmd = False
        self.reset_cmd = False

        self.ready_pub = self.create_publisher(Bool, "plc/robot_ready", 10)
        self.power_ready_pub = self.create_publisher(Bool, "plc/power_on_ready", 10)
        self.power_ready_alias_pub = self.create_publisher(Bool, "plc/power_ready", 10)
        self.enable_ready_pub = self.create_publisher(Bool, "plc/enable_ready", 10)
        self.power_off_ready_pub = self.create_publisher(
            Bool, "plc/power_off_ready", 10
        )
        self.disable_ready_pub = self.create_publisher(Bool, "plc/disable_ready", 10)
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
        self.create_service(Trigger, "plc/request_power_on", self._handle_request_power_on)
        self.create_service(Trigger, "plc/request_power_off", self._handle_request_power_off)
        self.create_service(Trigger, "plc/request_enable", self._handle_request_enable)
        self.create_service(Trigger, "plc/request_disable", self._handle_request_disable)
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
                        f"DB{self.db_number}.DBX{self.io_byte}.0~DBX{self.disable_ready_byte}.0"
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
        """Read PLC feedback bits from DB5.DBB8 and DB5.DBB9."""
        try:
            with self._plc_lock:
                status_len = max(self.io_byte, self.disable_ready_byte, self.estop_byte) - self.io_byte + 1
                data = self._plc.db_read(self.db_number, self.io_byte, status_len)

            power_ready = bool(get_bool(data, 0, self.power_on_ready_bit))
            enable_ready = bool(get_bool(data, 0, self.enable_ready_bit))
            power_off_ready = bool(get_bool(data, 0, self.power_off_ready_bit))
            disable_ready = bool(
                get_bool(data, self.disable_ready_byte - self.io_byte, self.disable_ready_bit)
            )
            if self.estop_byte >= self.io_byte and self.estop_bit >= 0:
                estop_status = bool(
                    get_bool(data, self.estop_byte - self.io_byte, self.estop_bit)
                )
            else:
                estop_status = False
            ready_status = power_ready and enable_ready and not estop_status

            # 当前阶段吸盘未接气泵，下面三个 topic 先保持 False，后续接气路再扩展。
            suction_feedback = False
            object_detected = False
            plc_alarm = False

            if (
                ready_status != self.robot_ready
                or power_ready != self.power_ready
                or enable_ready != self.enable_ready
                or power_off_ready != self.power_off_ready
                or disable_ready != self.disable_ready
                or estop_status != self.e_stop
                or suction_feedback != self.suction_feedback
                or object_detected != self.object_detected
                or plc_alarm != self.plc_alarm
            ):
                self.robot_ready = ready_status
                self.power_ready = power_ready
                self.enable_ready = enable_ready
                self.power_off_ready = power_off_ready
                self.disable_ready = disable_ready
                self.e_stop = estop_status
                self.suction_feedback = suction_feedback
                self.object_detected = object_detected
                self.plc_alarm = plc_alarm
                self.get_logger().info(
                    "PLC 状态更新: "
                    f"上电反馈已就绪={self.power_ready}, "
                    f"使能反馈已就绪={self.enable_ready}, "
                    f"下电反馈已就绪={self.power_off_ready}, "
                    f"下使能反馈已就绪={self.disable_ready}, "
                    f"机械臂就绪={self.robot_ready}, "
                    f"急停={self.e_stop}, "
                    f"吸盘输出反馈={self.suction_feedback}, "
                    f"吸住工件={self.object_detected}, "
                    f"PLC报警={self.plc_alarm}"
                )

            self.ready_pub.publish(Bool(data=ready_status))
            self.power_ready_pub.publish(Bool(data=power_ready))
            self.power_ready_alias_pub.publish(Bool(data=power_ready))
            self.enable_ready_pub.publish(Bool(data=enable_ready))
            self.power_off_ready_pub.publish(Bool(data=power_off_ready))
            self.disable_ready_pub.publish(Bool(data=disable_ready))
            self.estop_pub.publish(Bool(data=estop_status))
            self.suction_feedback_pub.publish(Bool(data=suction_feedback))
            self.object_detected_pub.publish(Bool(data=object_detected))
            self.plc_alarm_pub.publish(Bool(data=plc_alarm))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"读取 PLC 状态失败: {exc}")
            self._mark_disconnected()

    def _write_plc_command_bits(self) -> bool:
        """Write command bits to the configured PLC I/O byte."""
        if not self._ensure_connected():
            self.get_logger().error("PLC 未连接，无法写入指令。")
            return False

        try:
            with self._plc_lock:
                data = self._plc.db_read(self.db_number, self.io_byte, 1)
                set_bool(data, 0, self.power_on_request_bit, self.power_on_request)
                set_bool(data, 0, self.power_off_request_bit, self.power_off_request)
                set_bool(data, 0, self.enable_request_bit, self.enable_request)
                set_bool(data, 0, self.disable_request_bit, self.disable_request)
                set_bool(data, 0, self.suction_request_bit, self.suction_on_cmd)
                self._plc.db_write(self.db_number, self.io_byte, data)

            self.get_logger().info(
                "PLC 指令下发: "
                f"请求上电={self.power_on_request}, "
                f"请求下电={self.power_off_request}, "
                f"请求上使能={self.enable_request}, "
                f"请求下使能={self.disable_request}, "
                f"吸盘吸气={self.suction_on_cmd}, "
                f"破真空/放气={self.suction_release_cmd}, "
                f"复位={self.reset_cmd}, "
                f"写入地址=DB{self.db_number}.DBB{self.io_byte}"
            )
            return True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"写入 PLC 指令失败: {exc}")
            self._mark_disconnected()
            return False

    def _set_command_state(
        self,
        *,
        power_on: Optional[bool] = None,
        power_off: Optional[bool] = None,
        enable: Optional[bool] = None,
        disable: Optional[bool] = None,
        suction_on: Optional[bool] = None,
    ) -> bool:
        if power_on is not None:
            self.power_on_request = power_on
        if power_off is not None:
            self.power_off_request = power_off
        if enable is not None:
            self.enable_request = enable
        if disable is not None:
            self.disable_request = disable
        if suction_on is not None:
            self.suction_on_cmd = suction_on
        return self._write_plc_command_bits()

    def _request_power_on(self) -> bool:
        return self._set_command_state(power_on=True, power_off=False)

    def _request_power_off(self) -> bool:
        return self._set_command_state(power_on=False, power_off=True)

    def _request_enable(self) -> bool:
        return self._set_command_state(enable=True, disable=False)

    def _request_disable(self) -> bool:
        return self._set_command_state(enable=False, disable=True)

    def _handle_set_power_on(self, request: SetBool.Request, response: SetBool.Response):
        ok = self._request_power_on() if request.data else self._request_power_off()
        response.success = ok
        response.message = (
            "power_on request held" if request.data else "power_off request held"
        ) if ok else "failed to write power command"
        return response

    def _handle_set_enable(self, request: SetBool.Request, response: SetBool.Response):
        ok = self._request_enable() if request.data else self._request_disable()
        response.success = ok
        response.message = (
            "enable request held" if request.data else "disable request held"
        ) if ok else "failed to write enable command"
        return response

    def _handle_request_power_on(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        del request
        response.success = self._request_power_on()
        response.message = "power_on request held" if response.success else "failed"
        return response

    def _handle_request_power_off(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        del request
        response.success = self._request_power_off()
        response.message = "power_off request held" if response.success else "failed"
        return response

    def _handle_request_enable(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        del request
        response.success = self._request_enable()
        response.message = "enable request held" if response.success else "failed"
        return response

    def _handle_request_disable(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        del request
        response.success = self._request_disable()
        response.message = "disable request held" if response.success else "failed"
        return response

    def _handle_set_suction_on(self, request: SetBool.Request, response: SetBool.Response):
        ok = self._set_command_state(suction_on=request.data)
        response.success = ok
        response.message = "suction_on command written" if ok else "failed to write suction_on command"
        return response

    def _handle_set_suction_release(
        self, request: SetBool.Request, response: SetBool.Response
    ):
        self.suction_release_cmd = request.data
        ok = self._write_plc_command_bits()
        response.success = ok
        response.message = (
            "suction_release command written"
            if ok
            else "failed to write suction_release command"
        )
        return response

    def _handle_set_reset(self, request: SetBool.Request, response: SetBool.Response):
        self.reset_cmd = request.data
        ok = self._write_plc_command_bits()
        response.success = ok
        response.message = "reset command written" if ok else "failed to write reset command"
        return response

    def _handle_clear_commands(self, request: Trigger.Request, response: Trigger.Response):
        del request
        self.power_on_request = False
        self.power_off_request = False
        self.enable_request = False
        self.disable_request = False
        self.suction_on_cmd = False
        self.suction_release_cmd = False
        self.reset_cmd = False
        ok = self._write_plc_command_bits()
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
