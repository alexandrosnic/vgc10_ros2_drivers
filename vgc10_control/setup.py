from setuptools import setup

package_name = 'vgc10_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/bringup.launch.py',]),
        ('share/' + package_name + '/nodes', [
            'nodes/OnRobotVGSimpleController.py',
            'nodes/OnRobotVGStatusListener.py',
            'nodes/OnRobotVGTcpNode.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='alexnic31@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': ['launch_ros = launch_ros'],
        'console_scripts': [
            'OnRobotVGTcpNode = vgc10_control.OnRobotVGTcpNode:main',
            'OnRobotVGStatusListener = vgc10_control.OnRobotVGStatusListener:OnRobotVGStatusListener',
            'OnRobotVGSimpleController = vgc10_control.OnRobotVGSimpleController:publisher',
        ]
    },
)
