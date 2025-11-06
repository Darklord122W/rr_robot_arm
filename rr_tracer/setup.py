from setuptools import find_packages, setup

package_name = 'rr_tracer'

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
    maintainer='darklord',
    maintainer_email='25314403062@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'anglepublisher = rr_tracer.anglepublisher:main',
            'tf_tracer = rr_tracer.tf_tracer:main',
        ],
    },
)
