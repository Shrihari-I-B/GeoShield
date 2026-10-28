"""Gate the planner on GeoShield's verdict.

  ros2 launch /geoshield/launch/geoshield_gate.launch.py \
      candidate:=/geoshield/data/repro_width.osm \
      prior:=/autoware_map/nishishinjuku_autoware_map/lanelet2_map.osm

geoshield_verifier runs first. On exit 0 the admitted map is installed into
verified_dir and planning_simulator is launched against it. On any other exit
the planner is never started and launch shuts down.

autoware_launch is resolved inside the exit handler, not at setup, so the
refuse path does not require Autoware to be installed. That keeps the gate
testable on a machine with ROS but no Autoware.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

ARGS = (
    ("candidate", None, "map under test (.osm)"),
    ("prior", None, "last trusted map (.osm)"),
    ("verified_dir", "/autoware_map/geoshield_verified", "where the admitted map is installed"),
    ("geoshield_dir", "/geoshield", "repository root"),
    ("vehicle_model", "sample_vehicle", ""),
    ("sensor_model", "sample_sensor_kit", ""),
    ("repair", "true", "false refuses a failing map instead of repairing it"),
    ("hold", "2.0", "seconds the verifier keeps publishing its verdict"),
    ("flag_duration", "7200", "seconds the marker publisher stays up"),
)


def _setup(context, *_, **__):
    g = lambda n: LaunchConfiguration(n).perform(context)

    cmd = ["python3", "-u", str(Path(g("geoshield_dir")) / "geoshield_verifier.py"),
           "--candidate", g("candidate"), "--prior", g("prior"),
           "--out-dir", g("verified_dir"), "--publish", "--hold", g("hold")]
    if g("repair").strip().lower() in ("false", "0", "no", "off"):
        cmd.append("--no-repair")

    gate = ExecuteProcess(name="geoshield_gate", cmd=cmd, output="screen")

    def on_exit(event, ctx):
        if event.returncode == 0:
            sim = PathJoinSubstitution([
                FindPackageShare("autoware_launch"), "launch",
                "planning_simulator.launch.xml"]).perform(ctx)
            root = Path(g("geoshield_dir"))
            stem = Path(g("candidate")).stem
            return [
                LogInfo(msg="[GeoShield] map ADMITTED -- starting planner on "
                            + g("verified_dir")),
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(sim),
                    launch_arguments={
                        "map_path": g("verified_dir"),
                        "vehicle_model": g("vehicle_model"),
                        "sensor_model": g("sensor_model"),
                    }.items()),
                # The gate process has exited, and TRANSIENT_LOCAL durability
                # dies with its publisher, so the verdict it published is gone.
                # Keep a long-lived publisher alongside the planner instead.
                ExecuteProcess(
                    name="geoshield_flags",
                    cmd=["python3", "-u", str(root / "publish_flags.py"),
                         "--report", str(root / "results" / f"gate_{stem}_in.json"),
                         "--map", g("prior"), "--duration", g("flag_duration")],
                    output="log"),
            ]
        return [
            LogInfo(msg=f"[GeoShield] map REFUSED (gate exit "
                        f"{event.returncode}) -- planner NOT started"),
            Shutdown(reason="map refused by GeoShield"),
        ]

    return [gate, RegisterEventHandler(
        OnProcessExit(target_action=gate, on_exit=on_exit))]


def generate_launch_description():
    decls = [DeclareLaunchArgument(n, description=d)
             if dv is None else
             DeclareLaunchArgument(n, default_value=dv, description=d)
             for n, dv, d in ARGS]
    return LaunchDescription(decls + [OpaqueFunction(function=_setup)])
