from setuptools import find_packages, setup

package_name = 'bulldozer_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/bulldozer.launch.py']),
        ('share/' + package_name + '/config',
            ['config/bulldozer.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mfauzansyarif',
    maintainer_email='mfauzansyarif@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          'teleop_node = bulldozer_control.teleop_node:main',
          'serial_node = bulldozer_control.serial_node:main',
          'auto_node = bulldozer_control.auto_node:main',
          'selector_node = bulldozer_control.selector_node:main',
          'odom_node = bulldozer_control.odom_node:main',
        ],
    },
)
