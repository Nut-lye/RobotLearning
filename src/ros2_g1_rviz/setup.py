from glob import glob

from setuptools import setup

package_name = "ros2_g1_rviz"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        ("share/" + package_name + "/urdf", glob("urdf/*.urdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="a24",
    maintainer_email="a24@example.com",
    description="RViz2 visualization helpers for Unitree G1 joint state playback.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "demo_joint_state_publisher = ros2_g1_rviz.demo_joint_state_publisher:main",
            "lowstate_to_joint_state = ros2_g1_rviz.lowstate_to_joint_state:main",
            "sdk2_lowstate_to_joint_state = ros2_g1_rviz.sdk2_lowstate_to_joint_state:main",
        ],
    },
)
