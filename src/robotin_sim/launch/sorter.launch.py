import os
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start_simulation(context):
    options = {name: LaunchConfiguration(name).perform(context) for name in DEFAULTS}
    camera = options["camera"].lower() == "true"
    headless = options["headless"].lower() == "true"
    paused = options["paused"].lower() == "true" or (options["paused"] == "auto" and not headless)
    classes = [int(options[key]) for key in ("first_class", "second_class", "third_class")]
    while len(classes) > 1 and classes[-1] == -1:
        classes.pop()
    if any(label not in (0, 1, 2) for label in classes):
        raise ValueError("Las clases deben estar entre 0 y 2, o -1 para omitir frutos al final")
    if options["fixed_class"]:
        fixed_class = int(options["fixed_class"])
        if fixed_class not in (0, 1, 2):
            raise ValueError("fixed_class debe ser 0, 1 o 2")
        classes = [fixed_class] * len(classes)
    else:
        fixed_class = None
    model = options["model"]
    if model:
        model = str(Path(model).expanduser().resolve(strict=True))
        if not camera or fixed_class is not None:
            raise ValueError("La inferencia requiere cámara activa y canal variable")

    instance = f"sim_{os.getpid()}"
    scope = f"/robotin/{instance}"
    partition = os.environ.get("GZ_PARTITION") or f"robotin_{instance}"
    share = Path(get_package_share_directory("robotin_sim"))
    scenario = TemporaryDirectory(prefix="robotin-world-")
    world = Path(scenario.name) / "robotin.sdf"
    generator = [
        sys.executable, str(share / "scripts" / "make_world.py"), str(world),
        "--count", str(len(classes)), "--speed", options["belt_speed_mm_s"],
        "--diameter", options["diameter_mm"], "--mass", options["mass_kg"],
    ]
    if camera:
        generator.append("--camera")
    if fixed_class is not None:
        generator.extend(["--fixed-angle", str((43, 0, -43)[fixed_class])])
    subprocess.run(generator, check=True)

    def cleanup(_context):
        scenario.cleanup()
        return []

    plugin_dir = Path(get_package_prefix("robotin_sim")) / "lib"
    plugin_paths = [str(plugin_dir)]
    if current_path := os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH"):
        plugin_paths.append(current_path)
    vendor = Path(get_package_prefix("gz_sim_vendor")) / "opt" / "gz_sim_vendor"
    simulator = str(vendor / "libexec" / "gz" / "sim10" / "gz-sim-main")
    command = [simulator, "-s"]
    if not paused:
        command.append("-r")
    if camera:
        command.append("--headless-rendering")
    gazebo = ExecuteProcess(
        cmd=[*command, str(world)],
        additional_env={"GZ_SIM_SYSTEM_PLUGIN_PATH": os.pathsep.join(plugin_paths),
                        "GZ_SIM_RESOURCE_PATH": os.pathsep.join(
                            filter(None, (scenario.name, os.environ.get("GZ_SIM_RESOURCE_PATH")))
                        )},
        output="both",
    )
    topics = [
        "/robotin/plant@std_msgs/msg/String[gz.msgs.StringMsg",
        "/robotin/diverter/command@std_msgs/msg/Float64]gz.msgs.Double",
    ]
    if camera:
        topics.append("/robotin/camera/image@sensor_msgs/msg/Image[gz.msgs.Image")
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", namespace=scope,
                  arguments=topics, output="both", remappings=[
                      (f"/robotin/{topic}", f"{scope}/{topic}") for topic in (
                          "plant", "diverter/command", "camera/image"
                      )
                  ])
    sorter = Node(package="robotin_sim", executable="sorter.py", namespace=scope, output="both",
                  parameters=[{"camera": camera, "classes": classes, "model": model,
                               "scope": scope,
                               "diameter_mm": float(options["diameter_mm"])}])
    processes = [
        SetEnvironmentVariable("GZ_PARTITION", partition),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        RegisterEventHandler(OnProcessExit(target_action=gazebo, on_exit=[
            EmitEvent(event=Shutdown(reason="El servidor de Gazebo se cerró")),
        ])),
        gazebo, bridge, sorter,
    ]
    if not headless:
        gui_config = share / "config" / "robotin.gui.config"
        processes.append(ExecuteProcess(
            cmd=[simulator, "-g", "--gui-config", str(gui_config)],
            output="both",
        ))
        if options["viewer"].lower() == "true":
            processes.append(Node(
                package="robotin_sim", executable="viewer.py", output="both",
                namespace=scope,
                parameters=[{"camera": camera, "scope": scope}],
            ))
    return processes


DEFAULTS = {
    "headless": "false",
    "camera": "true",
    "paused": "auto",
    "viewer": "true",
    "first_class": "0",
    "second_class": "2",
    "third_class": "1",
    "belt_speed_mm_s": "100",
    "diameter_mm": "64",
    "mass_kg": "0.25",
    "fixed_class": "",
    "model": "",
}


def generate_launch_description():
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in DEFAULTS.items()),
        OpaqueFunction(function=start_simulation),
    ])
