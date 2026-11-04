from setuptools import find_packages, setup

package_name = 'sts3215_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'ftservo-python-sdk'],
    zip_safe=True,
    maintainer='UoM Arm Project',
    maintainer_email='todo@todo.todo',
    description='STS3215 velocity-mode servo driver (ftservo-python-sdk wrapper).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'driver_node = sts3215_driver.driver_node:main',
        ],
    },
)
