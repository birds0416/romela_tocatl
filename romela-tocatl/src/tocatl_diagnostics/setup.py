from setuptools import setup

package_name = "tocatl_diagnostics"

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
    description="Offline-safe TOCATL hardware inventory tools",
    license="MIT",
    entry_points={
        "console_scripts": [
            "tocatl-inventory = tocatl_diagnostics.motor_map:main",
            "tocatl-pi-inventory = tocatl_diagnostics.pi_inventory:main",
            "tocatl-telemetry = tocatl_diagnostics.cybergear_telemetry:main",
        ]
    },
)
