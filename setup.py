from setuptools import setup

package_name = 'seagraves_unmanned_systems_pkg'
support_moudle = 'support_module'
path_follower = 'seagraves_unmanned_systems_pkg/PathFollower'
search_algorithms = 'seagraves_unmanned_systems/SearchAlgorithms'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, support_moudle, path_follower, search_algorithms],
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
            'multi_bots = seagraves_unmanned_systems_pkg.multi_bots:main',
            'path_follower = seagraves_unmanned_systems_pkg.PathFollower.path_follower:main',
        ],
    },
)
