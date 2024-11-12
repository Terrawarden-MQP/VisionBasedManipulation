from setuptools import find_packages, setup

package_name = 'grasp_vision'

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
    maintainer='Mark Caleca',
    maintainer_email='macaleca@wpi.edu',
    description='Terrawarden Grasp Vision Pipeline',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'extract_cluster = grasp_vision.extract_cluster:main'
        ],
    },
)
