from setuptools import setup

package_name = 'seagraves_unmanned_systems_pkg'
path_follower = 'seagraves_unmanned_systems_pkg/PathFollower'
search_algorithms = 'SearchAlgorithms'
support_moudle = 'support_module'
tag_your_it = 'seagraves_unmanned_systems_pkg/TagYourIt'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name, 
        path_follower, 
        search_algorithms, 
        support_moudle, 
        tag_your_it
    ],
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
            'pursuer = seagraves_unmanned_systems_pkg.TagYourIt.pursuer:main',
        ],
    },
)
