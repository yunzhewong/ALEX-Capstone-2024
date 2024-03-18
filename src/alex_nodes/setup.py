from setuptools import find_packages, setup

package_name = "alex_nodes"

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
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "encoder_subscriber = alex_nodes.encoder_subscriber:main",
            "torque_publisher = alex_nodes.torque_publisher:main",
            "motor_pubsub = alex_nodes.motor_pubsub:main",
        ],
    },
)
