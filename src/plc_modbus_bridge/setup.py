from setuptools import find_packages, setup

package_name = "plc_modbus_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pymodbus>=3.0.0"],
    zip_safe=True,
    maintainer="lsh953",
    maintainer_email="3316594954@qq.com",
    description="ROS 2 Modbus TCP server bridge for Siemens PLC coordination.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "plc_modbus_bridge = plc_modbus_bridge.plc_modbus_bridge:main",
        ],
    },
)
