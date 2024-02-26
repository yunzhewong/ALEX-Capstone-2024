from setuptools import find_packages, setup

package_name = 'torque_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yun Zhe Wong',
    maintainer_email='yunzhe.wong@gmail.com',
    description='Publisher node to control the actuators on the ALEX Capstone Project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'torque_publisher = torque_publisher.torque_publisher:main',
        ],
    },
)
