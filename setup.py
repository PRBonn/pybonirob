from setuptools import setup

setup(
    name='pybonirob',
    version='1.0',
    license='BSD-2-Clause',
    author='Nived Chebrolu, Cyrill Stachniss',
    author_email='nived.chebrolu@uni-bonn.de',
    description='A basic set of tools to parse and access bonirob data.',
    packages=['pybonirob'],
    install_requires=['numpy', 'pyyaml', 'pillow', 'opencv-python']
)
