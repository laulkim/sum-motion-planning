from setuptools import find_packages, setup

package_name = "simp_tracker"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README_KR.md"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Jaepoong Lee",
    maintainer_email="ske03005@cbnu.ac.kr",
    description="Paper-based omnidirectional trajectory tracking prototype.",
    license="Proprietary",
    entry_points={"console_scripts": [
        "tracking_controller_node = simp_tracker.tracking_controller_node:main",
    ]},
)
