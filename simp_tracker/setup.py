from setuptools import find_packages, setup

package_name = "simp_tracker"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Jaepoong Lee",
    maintainer_email="ske03005@cbnu.ac.kr",
    description="Reference executor: samples the planner's published execution "
                "trajectory and forwards it to /cmd_vel, with no position feedback controller.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "tracker_node = simp_tracker.tracker_node:main",
        ],
    },
)
