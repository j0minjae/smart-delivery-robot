from setuptools import find_packages, setup

package_name = 'ssts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['ssts']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/database', ['database/database.db']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pgt',
    maintainer_email='rbxorns1@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'GUI = ssts.GUI:main',
            'unit_test = ssts.unit_test:main'
        ],
    },
)
