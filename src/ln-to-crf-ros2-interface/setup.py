import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ln_to_crf_ros2_interface"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*launch.py")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*.STL")),
        (os.path.join("share", package_name, "meshes", "collision"), glob("meshes/collision/*.stl")),
        (os.path.join("share", package_name, "meshes", "visual"), glob("meshes/visual/*.stl")),
        (os.path.join("share", package_name, "model"), glob("model/*.urdf")),
        (os.path.join("share", package_name, "model"), glob("model/*.xacro")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fior_eo",
    maintainer_email="Abhishek.Padalkar@dlr.de",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    package_data={"": ["*.yaml"]},
    entry_points={
        "console_scripts": ["topic_manager = ln_to_crf_ros2_interface.topic_manager:main",
                            "service_manager = ln_to_crf_ros2_interface.service_manager:main",
                            "fake_pub = ln_to_crf_ros2_interface.fake_publisher:main"],
    },
)
