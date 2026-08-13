from glob import glob
from os.path import join

from setuptools import find_packages, setup

package_name = "simp_planner_tools"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README_KR.md"]),
        (join("share", package_name, "maps"), glob("maps/*.csv")),
        (join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jaepoong Lee",
    maintainer_email="ske03005@cbnu.ac.kr",
    description="Input scenario and debug tools for the native C++ SIMP planner.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "scenario_manager_node = simp_planner_tools.scenario_manager_node:main",
            "track_map_provider_node = simp_planner_tools.track_map_provider_node:main",
            "debug_plot_node = simp_planner_tools.debug_plot_node:main",
            "planning_call_count_report_node = simp_planner_tools.planning_call_count_report:main",
        ],
    },
)
