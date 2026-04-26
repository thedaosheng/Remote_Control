from setuptools import setup

package_name = "teleop_chassis_safety"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    # YAML error-code files live inside the package so diag modules can load
    # them via Path(__file__).parent. package_data tells setuptools to bundle
    # them when the package is installed (colcon build → install/.../site-packages).
    package_data={
        package_name: ["*.yaml"],
    },
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/chassis_safety.yaml"]),
        (f"share/{package_name}/systemd", ["systemd/chassis-safety.service"]),
    ],
    install_requires=["setuptools", "python-can", "pyyaml"],
    zip_safe=True,
    maintainer="YZL",
    maintainer_email="yzl@cangjie.ai",
    description="仓颉/Faber 底盘 safety 节点 (watchdog + 风车 state machine)",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"chassis_safety_node = {package_name}.chassis_safety_node:main",
        ],
    },
)
