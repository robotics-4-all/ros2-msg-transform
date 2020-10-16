#!/usr/bin/env python

"""The setup script."""

from setuptools import setup, find_packages

with open('README.md') as readme_file:
    readme = readme_file.read()

# with open('HISTORY.rst') as history_file:
#     history = history_file.read()

# requirements = ['rosidl_runtime_py', 'rosidl_parser']
requirements = [ ]

setup_requirements = [ ]

test_requirements = [ ]

setup(
    author="Konstantinos Panayiotou",
    author_email='klpanagi@gmail.com',
    python_requires='>=3.5',
    classifiers=[
        'Development Status :: 2 - Pre-Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Natural Language :: English',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
    ],
    description="Methods to manipulate ROS2 messages",
    entry_points={},
    install_requires=requirements,
    license="MIT license",
    long_description=readme,
    include_package_data=True,
    keywords='ros2',
    name='ros2_msg_conv',
    packages=find_packages(include=['ros2_msg_conv', 'ros2_msg_conv.*']),
    setup_requires=setup_requirements,
    test_suite='tests',
    tests_require=test_requirements,
    url='https://github.com/robotics4all/ros2_msg_conv',
    version='0.2.2',
    zip_safe=False,
)

