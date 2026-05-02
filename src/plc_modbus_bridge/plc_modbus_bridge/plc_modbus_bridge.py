#!/usr/bin/env python3
"""ROS 2 Modbus TCP server bridge for Siemens S7-1214C PLC control."""

import asyncio
import threading
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

try:
    from pymodbus.constants import Endian
    from pymodbus.datastore import ModbusSequentialDataBlock, ModbusServerContext
    try:
        # pymodbus 3.x recent releases renamed SlaveContext to DeviceContext.
        from pymodbus.datastore import ModbusDeviceContext as ModbusSlaveContext
    except ImportError:
        from pymodbus.datastore import ModbusSlaveContext
    from pymodbus.payload import BinaryPayloadDecoder
    from pymodbus.server import StartAsyncTcpServer
except ImportError as exc:
    raise ImportError(
        "缺少 pymodbus 3.x。请先安装: python3 -m pip install 'pymodbus>=3.0.0'"
    ) from exc


def _endian_big():
    """兼容 pymodbus 3.x 不同小版本里的 Endian 命名。"""
    if hasattr(Endian, "BIG"):
        return Endian.BIG
    return Endian.Big


class LockedSequentialDataBlock(ModbusSequentialDataBlock):
    """带锁的 Holding Register 数据块。

    Modbus Server 和 ROS2 定时器运行在不同线程里，同时读写寄存器时需要互斥。
    """

    def __init__(self, address: int, values: List[int], lock: threading.RLock):
        super().__init__(address, values)
        self._lock = lock

    def getValues(self, address, count=1):  # noqa: N802, pymodbus API uses camelCase
        with self._lock:
            return super().getValues(address, count)

    def setValues(self, address, values):  # noqa: N802, pymodbus API uses camelCase
        with self._lock:
            return super().setValues(address, values)


class PlcModbusBridge(Node):
    """ROS2 节点: 作为 Modbus TCP Server, 等待 Siemens PLC 写入目标点。"""

    CONTROL_WORD_ADDR = 0
    TARGET_X_ADDR = 1
    TARGET_Y_ADDR = 3
    TARGET_Z_ADDR = 5
    STATUS_WORD_ADDR = 19

    STATUS_IDLE = 0
    STATUS_MOVING = 1
    STATUS_REACHED = 2

    CONTROL_IDLE = 0
    CONTROL_TRIGGER = 1

    def __init__(self) -> None:
        super().__init__("plc_modbus_bridge")

        self.declare_parameter("listen_host", "0.0.0.0")
        self.declare_parameter("listen_port", 502)
        self.declare_parameter("register_count", 100)
        self.declare_parameter("poll_hz", 10.0)
        self.declare_parameter("simulated_motion_seconds", 2.0)

        self.listen_host = self.get_parameter("listen_host").value
        self.listen_port = int(self.get_parameter("listen_port").value)
        self.register_count = int(self.get_parameter("register_count").value)
        self.simulated_motion_seconds = float(
            self.get_parameter("simulated_motion_seconds").value
        )

        self._register_lock = threading.RLock()
        self._previous_control_word = self.CONTROL_IDLE
        self._motion_active = False
        self._motion_lock = threading.Lock()
        self._server_exception: Optional[BaseException] = None

        self._holding_registers = LockedSequentialDataBlock(
            0, [0] * self.register_count, self._register_lock
        )
        self._slave_context = self._create_slave_context(self._holding_registers)
        self._server_context = self._create_server_context(self._slave_context)

        self._write_register(self.STATUS_WORD_ADDR, self.STATUS_IDLE)

        self._server_thread = threading.Thread(
            target=self._run_modbus_server_thread,
            name="modbus_tcp_server",
            daemon=True,
        )
        self._server_thread.start()

        poll_hz = float(self.get_parameter("poll_hz").value)
        self._timer = self.create_timer(1.0 / poll_hz, self._poll_control_word)

        self.get_logger().info(
            f"PLC Modbus Bridge 已启动: {self.listen_host}:{self.listen_port}, "
            f"Holding Registers={self.register_count}"
        )
        if self.listen_port < 1024:
            self.get_logger().warn(
                "端口 502 是 Linux 低端口，普通用户可能无权限监听。"
                "如果启动失败，可临时改用 1502 测试，或使用 sudo/setcap 配置权限。"
            )

    def _create_slave_context(self, holding_registers: LockedSequentialDataBlock):
        """创建 0 地址偏移的 Modbus 数据上下文。

        Siemens PLC 侧使用 40001 对应 Address 0，本节点内部也按 0 地址偏移处理。
        """
        try:
            return ModbusSlaveContext(hr=holding_registers, zero_mode=True)
        except TypeError:
            # 部分 pymodbus 3.x 版本取消了 zero_mode 参数。
            return ModbusSlaveContext(hr=holding_registers)

    def _create_server_context(self, slave_context):
        """创建 ServerContext, 兼容 pymodbus 3.x 的参数命名变化。"""
        try:
            return ModbusServerContext(slaves=slave_context, single=True)
        except TypeError:
            return ModbusServerContext(devices=slave_context, single=True)

    def _run_modbus_server_thread(self) -> None:
        """在后台线程中运行 asyncio Modbus TCP Server。"""
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self._start_modbus_server())
        except PermissionError as exc:
            self._server_exception = exc
            self.get_logger().error(
                f"Modbus TCP Server 启动失败: 无权限监听端口 {self.listen_port}。"
                "请使用 sudo、setcap，或把 listen_port 改为 1502 测试。"
            )
        except OSError as exc:
            self._server_exception = exc
            self.get_logger().error(
                f"Modbus TCP Server 启动失败: {exc}. "
                "请检查端口是否被占用、IP 是否可绑定。"
            )
        except Exception as exc:  # noqa: BLE001, server thread must not die silently
            self._server_exception = exc
            self.get_logger().exception(f"Modbus TCP Server 异常退出: {exc}")

    async def _start_modbus_server(self) -> None:
        self.get_logger().info(
            f"正在监听 Modbus TCP: {self.listen_host}:{self.listen_port}"
        )
        await StartAsyncTcpServer(
            context=self._server_context,
            address=(self.listen_host, self.listen_port),
        )

    def _poll_control_word(self) -> None:
        """10Hz 轮询 Control Word, 检测 0 -> 1 上升沿。"""
        if self._server_exception is not None:
            return

        try:
            control_word = self._read_register(self.CONTROL_WORD_ADDR)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"读取 Control Word 失败: {exc}")
            return

        rising_edge = (
            self._previous_control_word == self.CONTROL_IDLE
            and control_word == self.CONTROL_TRIGGER
        )
        self._previous_control_word = control_word

        if not rising_edge:
            return

        with self._motion_lock:
            if self._motion_active:
                self.get_logger().warn("收到触发信号，但机器人仍在运动中，本次触发被忽略。")
                return
            self._motion_active = True

        worker = threading.Thread(
            target=self._handle_motion_request,
            name="robot_motion_worker",
            daemon=True,
        )
        worker.start()

    def _handle_motion_request(self) -> None:
        """处理 PLC 的一次运动请求。"""
        try:
            x, y, z = self._read_target_xyz()
            self._write_register(self.STATUS_WORD_ADDR, self.STATUS_MOVING)
            self.get_logger().info(
                f"收到 PLC 目标坐标: X={x:.6f}, Y={y:.6f}, Z={z:.6f}"
            )

            self.execute_robot_motion(x, y, z)

            self._write_register(self.STATUS_WORD_ADDR, self.STATUS_REACHED)
            self.get_logger().info("机器人运动完成，Status Word=2(已到位)")
        except Exception as exc:  # noqa: BLE001
            self._write_register(self.STATUS_WORD_ADDR, self.STATUS_IDLE)
            self.get_logger().exception(f"执行 PLC 运动请求失败: {exc}")
        finally:
            with self._motion_lock:
                self._motion_active = False

    def _read_target_xyz(self) -> Tuple[float, float, float]:
        x = self._read_float32_big_endian(self.TARGET_X_ADDR)
        y = self._read_float32_big_endian(self.TARGET_Y_ADDR)
        z = self._read_float32_big_endian(self.TARGET_Z_ADDR)
        return x, y, z

    def _read_float32_big_endian(self, address: int) -> float:
        """读取两个 Holding Registers 并按 Siemens Big-Endian Float32 解码。"""
        registers = self._read_registers(address, 2)
        decoder = BinaryPayloadDecoder.fromRegisters(
            registers,
            byteorder=_endian_big(),
            wordorder=_endian_big(),
        )
        return float(decoder.decode_32bit_float())

    def _read_register(self, address: int) -> int:
        return int(self._read_registers(address, 1)[0])

    def _read_registers(self, address: int, count: int) -> List[int]:
        return self._slave_context.getValues(3, address, count)

    def _write_register(self, address: int, value: int) -> None:
        self._slave_context.setValues(3, address, [int(value) & 0xFFFF])

    def execute_robot_motion(self, x: float, y: float, z: float) -> None:
        """机器人运动占位接口。

        后续可在这里接入 MoveIt2，例如:
        - 根据 x/y/z 构造目标 Pose
        - 调用 MoveGroupInterface 或 action/service
        - 等待规划与执行完成
        """
        self.get_logger().info(
            f"execute_robot_motion 占位执行: x={x:.6f}, y={y:.6f}, z={z:.6f}"
        )
        time.sleep(self.simulated_motion_seconds)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlcModbusBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C，正在退出 plc_modbus_bridge。")
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
