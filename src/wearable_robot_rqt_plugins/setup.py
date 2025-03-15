from setuptools import setup
import os
from glob import glob

package_name = 'wearable_robot_rqt_plugins'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
        # UI 파일을 패키지 자체의 리소스 디렉토리에 직접 설치합니다
        (os.path.join('lib', package_name, 'resource'), glob('resource/*.ui')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='웨어러블 로봇 시스템을 위한 rqt 플러그인 모음',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'actuator_control_plugin = wearable_robot_rqt_plugins.actuator_control_plugin:main',
            'waist_control_plugin = wearable_robot_rqt_plugins.waist_control_plugin:main',
        ],
    },
)
