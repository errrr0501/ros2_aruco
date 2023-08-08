from setuptools import setup

package_name = 'ros2_aruco'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nathan Sprague',
    maintainer_email='nathan.r.sprague@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mask_color_and_depth_image = ros2_aruco.mask_color_and_depth_image:main',
            'aruco_generate_marker = ros2_aruco.aruco_generate_marker:main',
            'test_mask = ros2_aruco.test_mask:main',
            'hiwin_cali = ros2_aruco.three_points_cali_aruco:main',
            'tm_aruco = ros2_aruco.single_aruco_set_tf:main',
        ],
    },
)
