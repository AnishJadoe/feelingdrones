from setuptools import setup

package_name = 'mpr121_pubsub'

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
    maintainer='anish',
    maintainer_email='ajadoe@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = mpr121_pubsub.publisher_function:main',
            'listener = mpr121_pubsub.subscriber_function:main',
            
        ],
    },
)
