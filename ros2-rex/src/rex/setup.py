from setuptools import setup, find_packages

package_name = 'rex'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['rex/detect_continuous.py', 'rex/human_detect.py', 'rex/tracklets_parser.py', 'rex/controller.py', 'rex/control_service.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Tracklets parser for DepthAI tracklets',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracklets_parser = rex.tracklets_parser:main',
            'controller = rex.controller:main',
            'control_service = rex.control_service:main',
            'human_detect = rex.human_detect:main',
            'detect_continuous = rex.detect_continuous:main',
        ],
    },
)
