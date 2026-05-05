from setuptools import find_packages, setup

package_name = "plc_s7_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "python-snap7>=1.3"],
    zip_safe=True,
    maintainer="lsh953",
    maintainer_email="3316594954@qq.com",
    description="ROS 2 bridge for Siemens S7 PLC DB bit control using python-snap7.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "plc_s7_bridge = plc_s7_bridge.plc_s7_bridge:main",
        ],
    },
)
