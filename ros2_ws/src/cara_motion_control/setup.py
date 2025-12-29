from setuptools import setup

package_name = "cara_motion_control"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        # If you later add launch files:
        # (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="elida_z",
    maintainer_email="",
    description="Cara motion/hardware control nodes (head expression, IMU, ros2_control bridges).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "head_expression_node = cara_motion_control.head_expression_node:main",
        ],
    },
)

