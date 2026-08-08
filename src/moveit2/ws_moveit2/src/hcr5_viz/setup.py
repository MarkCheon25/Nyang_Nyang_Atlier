import os
from glob import glob

from setuptools import find_packages, setup

package_name = "hcr5_viz"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ysh",
    maintainer_email="yusungho1534@gmail.com",
    description="HCR-5 펜 끝 자취 시각화 (읽기 전용)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # ros2 run hcr5_viz pen_trail
            "pen_trail = hcr5_viz.pen_trail:main",
        ],
    },
)
