from setuptools import setup

package_name = 'amr_benchmark'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='devesh',
    maintainer_email='devesh@todo.todo',
    description='Benchmarking tools for AMR localization drift',
    license='MIT',
)
