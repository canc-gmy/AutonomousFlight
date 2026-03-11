from setuptools import setup

package_name = 'depth_camera_viewer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='simone',
    maintainer_email='simone@todo.todo',
    description='Depth camera viewer',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'depth_camera_viewer = depth_camera_viewer.depth_camera_viewer_node:main',
        ],
    },
)
