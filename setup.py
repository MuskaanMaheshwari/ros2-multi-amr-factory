from setuptools import setup, find_packages

with open("requirements.txt") as f:
    requirements = f.read().splitlines()

setup(
    name="ros2-multi-amr-factory",
    version="1.0.0",
    author="Muskaan Maheshwari",
    description="Multi-AMR Fleet Navigation for EV/Battery Factory Environments",
    url="https://github.com/muskaanmaheshwari/ros2-multi-amr-factory",
    license="MIT",
    python_requires=">=3.8",
    install_requires=requirements,
    entry_points={
        "console_scripts": [
            "ros2-factory-nav=main:main",
        ],
    },
)
