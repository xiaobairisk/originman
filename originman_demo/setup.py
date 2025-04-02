from setuptools import setup

package_name = 'originman_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'opencv-python', 'numpy', 'smbus2'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='liqiaolong@guyuehome.com',
    description='Originman demo package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rgb_ring_node = originman_demo.rgb_ring_node:main',
            'imu_print_node = originman_demo.imu_print_node:main',
            'oled_display_node = originman_demo.oled_display_node:main',
            'image_display_node = originman_demo.image_display_node:main',
            'cmdvel_to_action_node = originman_demo.cmdvel_to_action_node:main',
        ],
    },
)