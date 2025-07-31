from setuptools import setup

package_name = 'go2_trot_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Simple quadruped trotting publisher for Isaac Sim',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trot_publisher = go2_trot_publisher.trot_publisher:main',
            'trot_pd_controller = go2_trot_publisher.trot_pd_controller:main',
            'walk_crawl_pd_controller = go2_trot_publisher.walk_crawl_pd_controller:main',
        ],
    },
)
