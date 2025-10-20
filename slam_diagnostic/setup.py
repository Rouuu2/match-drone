from setuptools import find_packages, setup

package_name = 'slam_diagnostic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),  # FIXED
    data_files=[
        ('share/ament_index/resource_index/packages',  # FIXED
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'open3d',
        'pandas',
    ],
    zip_safe=True,
    maintainer='Roua Guetat',
    maintainer_email='rouaguetat2020@gmail.com',
    description='Diagnostic and evaluation tools for SLAM debugging and metrics.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_eval_node = slam_diagnostic.slam_eval_node:main',
            'slam_fixers_node = slam_diagnostic.slam_fixers_node:main',
            'odom_bridge_node = slam_diagnostic.odom_bridge_node:main',
            'pcd_replay_node = slam_diagnostic.pcd_replay_node:main',
        ],
    },
)
