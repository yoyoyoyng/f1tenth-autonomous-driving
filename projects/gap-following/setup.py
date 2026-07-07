from setuptools import find_packages, setup

package_name = 'gap_following'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoyoyoyng',
    maintainer_email='example@example.com',
    description='F1TENTH LiDAR-based Gap Following with Disparity Extension',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gap_follow_node = gap_following.gap_follow_node:main',
        ],
    },
)
