from setuptools import setup

package_name = 'video_recorder'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raph',
    maintainer_email='your_email@example.com',
    description='Video recorder with detection overlays',
    license='MIT',
    entry_points={
        'console_scripts': [
            'recorder = video_recorder.video_recorder_node:main',
        ],
    },
)