from setuptools import find_packages, setup

package_name = 'echo_chamber'

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
    maintainer='garvi',
    maintainer_email='garvit.work07@gmail.com',
    description='Week 0 ROS 2 publisher and subscriber assignment.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'talker = echo_chamber.talker:main',
            'listener = echo_chamber.listener:main',
        ],
    },
)
