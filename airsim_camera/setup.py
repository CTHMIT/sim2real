from setuptools import find_packages, setup

package_name = "airsim_camera"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cthsu",
    maintainer_email="chuntsehsu@gmail.com",
    description="Unreal Engine AirSim camera receiver for ROS2",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rtsp_image_receiver = airsim_camera.rtsp_image_receiver:main"
        ],
    },
)
