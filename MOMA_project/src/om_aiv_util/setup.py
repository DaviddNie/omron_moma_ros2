import os
from glob import glob
from setuptools import setup

package_name = 'om_aiv_util'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gy',
    maintainer_email='e0310259@u.nus.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arcl_api_server = om_aiv_util.arcl_api_server:main',
            'ld_states_publisher = om_aiv_util.ld_states_publisher:main',
        ],
    },
)
