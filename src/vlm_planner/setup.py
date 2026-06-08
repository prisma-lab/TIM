from setuptools import find_packages, setup

package_name = "vlm_task_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/vlm_planner.launch.py"]),
    ],
    install_requires=["setuptools", "google-generativeai"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@todo.todo",
    description="VLM-grounded PDDL problem publisher",
    license="MIT",
    entry_points={
        "console_scripts": [
            "vlm_planner_node = vlm_task_planner.vlm_planner_node:main",
        ],
    },
)
