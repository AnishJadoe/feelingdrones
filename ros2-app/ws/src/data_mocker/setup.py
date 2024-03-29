from setuptools import find_packages, setup

package_name = 'data_mocker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anish',
    maintainer_email='ajadoe@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpr121_mocker = data_mocker.mpr121_mocker:main',
            'gripper_mocker = data_mocker.gripper_mocker:main',
            'gripper_open = data_mocker.open_gripper:main',
            'gripper_close = data_mocker.close_gripper:main',
            'demo_open_close = data_mocker.gripper_open_close:main'
        ],
    },
)
