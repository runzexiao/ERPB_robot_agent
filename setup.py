from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ERPB_robot_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         # 包含 launch 目录中的所有文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # 包含 config 目录中的所有文件
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='runzexiao@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publish_dictionaries = ERPB_robot_agent.publish_dictionaries:main',
            'subscribe_dictionaries = ERPB_robot_agent.subscribe_dictionaries:main',
            'start_node = ERPB_robot_agent.start_node:main',
            'bidding_node = ERPB_robot_agent.bidding_node:main',
            'bidding_test = ERPB_robot_agent.bidding_test:main',
            'test_node = ERPB_robot_agent.test_node:main',
            'operator_task_processing_node = ERPB_robot_agent.operator_task_processing_node:main',
        ],
    },
)
