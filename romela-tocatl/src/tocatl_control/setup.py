from setuptools import setup

package_name = "tocatl_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Matthew Hutchinson",
    maintainer_email="matthewhutchinson@ucla.edu",
    description="TOCATL safety state management and motor control scaffolding",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "state_manager = tocatl_control.state_manager:main",
            "tocatl-validate-calibration = tocatl_control.calibration:main",
        ],
    },
)
