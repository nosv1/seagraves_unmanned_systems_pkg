from setuptools import setup

package_name = 'seagraves_unmanned_systems_pkg'
support_moudle = 'support_module'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, support_moudle],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thomas',
    maintainer_email='christopher.seagraves@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = seagraves_unmanned_systems_pkg.controller:main',
        ],
    },
)
