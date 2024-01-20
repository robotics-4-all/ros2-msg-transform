#!/usr/bin/env python

"""The setup script."""

from setuptools import setup, find_packages

with open('README.md') as readme_file:
    readme = readme_file.read()

requirements = []

with open("requirements.txt") as f:
    requirements = f.read().splitlines()

setup(
    author="Konstantinos Panayiotou",
    author_email='klpanagi@ece.auth.gr',
    python_requires='>=3.7',
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Natural Language :: English',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
    ],
    description="Library to manipulate ROS2 messages",
    entry_points={},
    install_requires=requirements,
    license="MIT license",
    long_description=readme,
    include_package_data=True,
    keywords=['ros2', 'iot', 'cps'],
    name='ros2_msg_transform',
    packages=find_packages(include=['ros2_msg_transform', 'ros2_msg_transform.*']),
    url='https://github.com/robotics4all/ros2_msg_transform',
    version='0.2.2',
    zip_safe=False,
)

