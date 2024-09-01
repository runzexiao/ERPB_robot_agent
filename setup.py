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
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # 将所有Python文件和.env文件复制到lib/ERPB_robot_agent/目录下
        (os.path.join('lib', package_name), glob('ERPB_robot_agent/*function.py')),
        (os.path.join('lib', package_name), glob('ERPB_robot_agent/.env'))
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
            'start_node = ERPB_robot_agent.start_node:main',
            'bidding_node = ERPB_robot_agent.bidding_node:main',
            'bidding_test = ERPB_robot_agent.bidding_test:main',
            'operator_task_processing_node = ERPB_robot_agent.operator_task_processing_node:main',
            'environmental_information_publisher_node = ERPB_robot_agent.environmental_information_publisher_node:main',
            'task_decomposition_node = ERPB_robot_agent.task_decomposition_node:main',
            'bidding_evaluation_node = ERPB_robot_agent.bidding_evaluation_node:main',
            'task_manager_node = ERPB_robot_agent.task_manager_node:main',
            'execute_node = ERPB_robot_agent.execute_node:main',
            'task_broadcaster_node = ERPB_robot_agent.task_broadcaster_node:main'
        ],
    },
)
