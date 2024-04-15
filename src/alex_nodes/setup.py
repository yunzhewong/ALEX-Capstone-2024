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
            "motor_pubsub = alex_nodes.motor_pubsub:main",
            'motor_user_interface = alex_nodes.motor_user_interface:main',
            'encoder_reader = alex_nodes.encoder_reader:main',
        ],
    },
)
