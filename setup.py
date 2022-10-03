from setuptools import setup

setup(
    name='aerotechapi',
    version='0.0.1',
    author='Frederic Schell',
    author_email='frederic.jschell@gmail.com',
    
    description='API for the Aerotech A3200 controller',
    py_modules=['aerotechpi', 'utils'],
    package_dir={'': 'aerotechapi'},
    classifiers=[
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'License :: OSI Approved :: MIT License'
    ],
    python_requires='>=3.6'
)
