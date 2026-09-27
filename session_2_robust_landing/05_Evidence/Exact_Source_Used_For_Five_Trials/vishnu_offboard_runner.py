#!/usr/bin/env python3

import json
import math
import re
import subprocess
import threading
import time

from pymavlink import mavutil

from RandomizedPositionController import RandomizedPositionController
from spawn_config import build_pose, parser, quaternion_from_rpy_deg


PRESTREAM_SECONDS = 2.0
MODE_TIMEOUT = 8.0
ARM_TIMEOUT = 8.0
LAND_TIMEOUT = 40.0
MISSION_TIMEOUT = 220.0
RELEASE_SPOOL_VZ_M_S = -0.80
RELEASE_SPOOL_MIN_TARGET_THRUST = 0.62
RELEASE_SPOOL_MIN_MEAN_MOTOR_OUTPUT = 650.0
RELEASE_SPOOL_READY_HOLD_S = 0.15
RELEASE_SPOOL_TIMEOUT = 3.0
RELEASE_TELEMETRY_MAX_AGE_S = 0.25

PX4_MAIN_MODE_AUTO = 4
PX4_MAIN_MODE_OFFBOARD = 6
PX4_AUTO_SUB_MODE_LAND = 6


def snapshot(controller):
    with controller.state_lock:
        return controller.state.mode, controller.state.armed, controller.state.yaw


def heartbeat_snapshot(controller):
    with controller.state_lock:
        if not controller.state.heartbeat_received:
            return None, None, None
        return (
            controller.state.heartbeat_main_mode,
            controller.state.heartbeat_sub_mode,
            controller.state.armed,
        )


def position_snapshot(controller):
    with controller.state_lock:
        state = controller.state
        return (
            state.x,
            state.y,
            state.z,
            state.vx,
            state.vy,
            state.vz,
            state.yaw,
        )


def set_int_param_before_receiver(master, name, value, timeout=5.0):
    encoded_name = name.encode("ascii")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            encoded_name,
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        )
        window_end = min(deadline, time.monotonic() + 1.0)
        while time.monotonic() < window_end:
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.25)
            if msg is None:
                continue
            param_id = msg.param_id
            if isinstance(param_id, bytes):
                param_id = param_id.decode("ascii", errors="ignore")
            param_id = str(param_id).rstrip("\x00")
            if param_id == name:
                actual = int(round(float(msg.param_value)))
                if actual != int(value):
                    raise RuntimeError(
                        f"PX4 parameter {name} acknowledged as {actual}, expected {value}"
                    )
                print(f"[runner] PX4 parameter {name}={actual} confirmed")
                return
    raise RuntimeError(f"Timed out setting PX4 parameter {name}={value}")


def send_neutral(controller):
    _, _, yaw = snapshot(controller)
    controller.send_velocity(0.0, 0.0, 0.0, yaw)


def release_propulsion_snapshot(controller, now_mono=None):
    now_mono = time.monotonic() if now_mono is None else now_mono
    with controller.state_lock:
        state = controller.state
        thrust = float(state.attitude_target_thrust)
        motors = tuple(float(value) for value in state.actuator_outputs[:4])
        attitude_age = (
            math.inf
            if state.attitude_target_received_at is None
            else now_mono - state.attitude_target_received_at
        )
        actuator_age = (
            math.inf
            if state.actuator_received_at is None
            else now_mono - state.actuator_received_at
        )

    finite_motors = [value for value in motors if math.isfinite(value)]
    return {
        "target_thrust": thrust,
        "min_motor_output": (
            min(finite_motors) if len(finite_motors) == 4 else math.nan
        ),
        "mean_motor_output": (
            sum(finite_motors) / 4.0 if len(finite_motors) == 4 else math.nan
        ),
        "attitude_target_age_s": attitude_age,
        "actuator_age_s": actuator_age,
    }


def release_propulsion_ready(snapshot):
    return (
        math.isfinite(snapshot["target_thrust"])
        and snapshot["target_thrust"] >= RELEASE_SPOOL_MIN_TARGET_THRUST
        and math.isfinite(snapshot["mean_motor_output"])
        and snapshot["mean_motor_output"] >= RELEASE_SPOOL_MIN_MEAN_MOTOR_OUTPUT
        and snapshot["attitude_target_age_s"] <= RELEASE_TELEMETRY_MAX_AGE_S
        and snapshot["actuator_age_s"] <= RELEASE_TELEMETRY_MAX_AGE_S
    )


def spool_propulsion_for_release(controller, timeout=RELEASE_SPOOL_TIMEOUT):
    """Trigger PX4 takeoff spool while the fixture prevents any displacement."""
    started = time.monotonic()
    deadline = started + timeout
    ready_since = None
    last = release_propulsion_snapshot(controller, started)

    while time.monotonic() < deadline:
        main_mode, sub_mode, armed = heartbeat_snapshot(controller)
        if main_mode != PX4_MAIN_MODE_OFFBOARD or armed is not True:
            raise RuntimeError(
                "PX4 left armed OFFBOARD during release spool: "
                f"main_mode={main_mode} sub_mode={sub_mode} armed={armed}"
            )

        _, _, yaw = snapshot(controller)
        controller.send_velocity(0.0, 0.0, RELEASE_SPOOL_VZ_M_S, yaw)
        time.sleep(controller.control_dt)

        now_mono = time.monotonic()
        last = release_propulsion_snapshot(controller, now_mono)
        if release_propulsion_ready(last):
            if ready_since is None:
                ready_since = now_mono
            elif now_mono - ready_since >= RELEASE_SPOOL_READY_HOLD_S:
                result = dict(last)
                result.update(
                    {
                        "duration_s": now_mono - started,
                        "commanded_vz_m_s": RELEASE_SPOOL_VZ_M_S,
                        "ready_hold_s": now_mono - ready_since,
                    }
                )
                print(
                    "[runner] Release propulsion ready: "
                    f"thrust={last['target_thrust']:.3f} "
                    f"min_motor={last['min_motor_output']:.1f} "
                    f"mean_motor={last['mean_motor_output']:.1f} "
                    f"duration={result['duration_s']:.2f}s"
                )
                return result
        else:
            ready_since = None

    raise RuntimeError(
        "Propulsion did not reach safe release readiness before timeout: "
        f"{last}"
    )


def stream_for(controller, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        send_neutral(controller)
        time.sleep(controller.control_dt)


def neutral_stream_loop(controller, stop_event):
    while not stop_event.is_set():
        send_neutral(controller)
        stop_event.wait(controller.control_dt)


def wait_heartbeat_state(controller, timeout, predicate, keep_streaming=True):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if keep_streaming:
            send_neutral(controller)
        main_mode, sub_mode, armed = heartbeat_snapshot(controller)
        if predicate(main_mode, sub_mode, armed):
            return True
        time.sleep(controller.control_dt)
    main_mode, sub_mode, armed = heartbeat_snapshot(controller)
    return bool(predicate(main_mode, sub_mode, armed))


def request_mode(controller, mode_name):
    master = controller.master
    mode_name = mode_name.upper()
    with controller.mav_send_lock:
        if mode_name == "OFFBOARD":
            custom_mode = PX4_MAIN_MODE_OFFBOARD << 16
            master.mav.set_mode_send(
                master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode,
            )
            return
        if mode_name == "LAND":
            custom_mode = (
                (PX4_AUTO_SUB_MODE_LAND << 24)
                | (PX4_MAIN_MODE_AUTO << 16)
            )
            master.mav.set_mode_send(
                master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode,
            )
            return
        master.set_mode(mode_name)


def request_arm(controller, arm=True):
    with controller.mav_send_lock:
        controller.master.mav.command_long_send(
            controller.master.target_system,
            controller.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0 if arm else 0.0,
            0,
            0,
            0,
            0,
            0,
            0,
        )


def set_world_paused(world, paused):
    value = "true" if paused else "false"
    result = subprocess.run(
        [
            "gz",
            "service",
            "-s",
            f"/world/{world}/control",
            "--reqtype",
            "gz.msgs.WorldControl",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            f"pause: {value}",
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=8.0,
    )
    output = (result.stdout + "\n" + result.stderr).strip()
    if result.returncode != 0 or "true" not in output.lower():
        raise RuntimeError(
            f"Gazebo world {'pause' if paused else 'resume'} failed "
            f"(exit={result.returncode}): {output}"
        )
    print(f"[runner] Gazebo world {'paused' if paused else 'resumed'}")


def teleport_model(pose, world, model, roll_deg=None, pitch_deg=None):
    roll_deg = pose.roll_deg if roll_deg is None else roll_deg
    pitch_deg = pose.pitch_deg if pitch_deg is None else pitch_deg
    qx, qy, qz, qw = quaternion_from_rpy_deg(
        roll_deg, pitch_deg, pose.yaw_deg
    )
    request = (
        f'name: "{model}" '
        f"position {{ x: {pose.x:.9f} y: {pose.y:.9f} z: {pose.z:.9f} }} "
        "orientation { "
        f"x: {qx:.12f} y: {qy:.12f} z: {qz:.12f} w: {qw:.12f} "
        "}"
    )
    command = [
        "gz",
        "service",
        "-s",
        f"/world/{world}/set_pose",
        "--reqtype",
        "gz.msgs.Pose",
        "--reptype",
        "gz.msgs.Boolean",
        "--timeout",
        "5000",
        "--req",
        request,
    ]
    result = subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
        timeout=8.0,
    )
    output = (result.stdout + "\n" + result.stderr).strip()
    if result.returncode != 0 or "true" not in output.lower():
        raise RuntimeError(
            f"Gazebo set_pose failed (exit={result.returncode}): {output}"
        )
    print(f"[runner] Gazebo set_pose accepted for {model}: {output}")


def create_spawn_fixture(pose, world, model):
    # Hold the exact bounded initial pose through PX4 spool-up. Keeping the
    # pose fixed until release avoids an unarmed free fall; the conservative
    # default +/-2 deg tilt limits attitude-controller loading on the joint.
    roll = math.radians(pose.roll_deg)
    pitch = math.radians(pose.pitch_deg)
    yaw = math.radians(pose.yaw_deg)
    sdf = (
        "<sdf version='1.9'><model name='spawn_fixture'>"
        "<static>true</static>"
        f"<pose>{pose.x:.9f} {pose.y:.9f} {pose.z:.9f} "
        f"{roll:.12f} {pitch:.12f} {yaw:.12f}</pose>"
        "<link name='fixture_link'/>"
        "<plugin filename='gz-sim-detachable-joint-system' "
        "name='gz::sim::systems::DetachableJoint'>"
        "<parent_link>fixture_link</parent_link>"
        f"<child_model>{model}</child_model>"
        "<child_link>base_link</child_link>"
        "<detach_topic>/spawn_fixture/detach</detach_topic>"
        "<output_topic>/spawn_fixture/state</output_topic>"
        "</plugin></model></sdf>"
    )
    result = subprocess.run(
        [
            "gz",
            "service",
            "-s",
            f"/world/{world}/create",
            "--reqtype",
            "gz.msgs.EntityFactory",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            f'sdf: "{sdf}"',
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=8.0,
    )
    output = (result.stdout + "\n" + result.stderr).strip()
    if result.returncode != 0 or "true" not in output.lower():
        raise RuntimeError(
            f"Gazebo fixture creation failed (exit={result.returncode}): {output}"
        )
    print(f"[runner] Gazebo spawn fixture attached: {output}")


def gazebo_model_pose(model):
    result = subprocess.run(
        ["gz", "model", "-m", model, "-p"],
        check=False,
        capture_output=True,
        text=True,
        timeout=8.0,
    )
    if result.returncode != 0:
        raise RuntimeError(f"Could not query Gazebo model pose: {result.stderr.strip()}")
    match = re.search(
        r"Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:\s*\n\s*"
        r"\[([^\]]+)\]\s*\n\s*\[([^\]]+)\]",
        result.stdout,
    )
    if match is None:
        raise RuntimeError(f"Could not parse Gazebo model pose: {result.stdout}")
    xyz = tuple(float(value) for value in match.group(1).split())
    rpy = tuple(float(value) for value in match.group(2).split())
    return xyz + rpy


def verify_fixture_height(model, requested_height):
    time.sleep(0.5)
    pose = gazebo_model_pose(model)
    if abs(pose[2] - requested_height) > 0.75:
        raise RuntimeError(
            "Fixture did not hold the requested airborne height: "
            f"requested={requested_height:.3f} actual={pose[2]:.3f}"
        )
    print(
        "[runner] Fixture height verified in Gazebo: "
        f"z={pose[2]:.3f}m requested={requested_height:.3f}m"
    )
    return pose


def detach_spawn_fixture():
    result = subprocess.run(
        [
            "gz",
            "topic",
            "-t",
            "/spawn_fixture/detach",
            "-m",
            "gz.msgs.Empty",
            "-p",
            "unused: true",
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=5.0,
    )
    if result.returncode != 0:
        raise RuntimeError(f"Gazebo fixture detach failed: {result.stderr.strip()}")
    print("[runner] Gazebo spawn fixture detached")


def wait_for_airborne_estimator_settle(controller, requested_height, timeout=12.0):
    # A teleport can cause PX4's estimator to reset/rebase its local origin.
    # That is the ambiguity this experiment is designed to handle, so do not
    # require local Z to equal the Gazebo world height. The accepted set_pose
    # response verifies the simulation move; here we only require the new PX4
    # local state to become finite and dynamically quiet before capturing it.
    deadline = time.monotonic() + timeout
    samples = []
    while time.monotonic() < deadline:
        current = position_snapshot(controller)
        if all(math.isfinite(value) for value in current):
            samples.append(current)
        samples = samples[-25:]
        if len(samples) >= 25:
            x_span = max(row[0] for row in samples) - min(row[0] for row in samples)
            y_span = max(row[1] for row in samples) - min(row[1] for row in samples)
            z_span = max(row[2] for row in samples) - min(row[2] for row in samples)
            last = samples[-1]
            quiet = (
                x_span <= 0.50
                and y_span <= 0.50
                and z_span <= 0.35
                and math.hypot(last[3], last[4]) <= 1.0
                and abs(last[5]) <= 0.75
            )
            if quiet:
                print(
                    "[runner] Airborne PX4 estimator settled after Gazebo move: "
                    f"local=({last[0]:.3f},{last[1]:.3f},{last[2]:.3f}) "
                    f"launcher_height={requested_height:.3f}m"
                )
                return last
        time.sleep(0.05)
    raise RuntimeError(
        "PX4 local state did not settle after randomized initialization; "
        f"requested_height={requested_height:.3f} last_samples={samples[-3:]}"
    )


def write_metadata(path, payload):
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")


def main(argv=None):
    args = parser().parse_args(argv)
    pose = build_pose(args)
    pose_dict = pose.as_dict()
    pose_dict.update({"world": args.world, "model": args.model})
    write_metadata("spawn_config.json", {"requested": pose_dict})
    print("[runner] Randomized initialization:")
    print(json.dumps(pose_dict, indent=2, sort_keys=True))

    c = RandomizedPositionController(
        spawn_height_m=pose.z,
        spawn_pose=pose_dict,
    )
    worker = None
    neutral_thread = None
    neutral_stop = threading.Event()
    fixture_created = False
    fixture_detached = False
    world_paused = False
    release_spool = None
    completed = False
    controller_stopped = False

    try:
        print("[runner] Connecting to PX4 SITL...")
        c.connect()
        set_int_param_before_receiver(c.master, "NAV_DLL_ACT", 0)
        set_int_param_before_receiver(c.master, "SDLOG_MODE", 0)

        c.start_receiver()
        time.sleep(1.0)
        c.wait_for_fresh_telemetry()
        ground_x, ground_y, ground_z, _, _, _, ground_yaw = position_snapshot(c)
        c.ground_reference = (ground_x, ground_y, ground_z, ground_yaw)
        print(
            "[runner] Ground reference captured before airborne initialization: "
            f"x={ground_x:.3f} y={ground_y:.3f} z={ground_z:.3f} "
            f"yaw={ground_yaw:.3f}"
        )

        main_mode, sub_mode, armed = heartbeat_snapshot(c)
        print(
            f"[runner] Initial heartbeat main_mode={main_mode} "
            f"sub_mode={sub_mode} armed={armed}"
        )
        if armed:
            raise RuntimeError("PX4 unexpectedly armed before randomized initialization")

        # Establish and estimate the bounded airborne pose while disarmed. This
        # prevents the PX4 velocity/rate controllers from loading against the
        # rigid fixture during the estimator-settling interval.
        print("[runner] Moving disarmed vehicle into randomized airborne fixture...")
        # Freeze physics across the two Gazebo service calls so the vehicle
        # cannot fall between set_pose and detachable-joint attachment.
        set_world_paused(args.world, True)
        world_paused = True
        try:
            teleport_model(pose, args.world, args.model)
            create_spawn_fixture(pose, args.world, args.model)
            fixture_created = True
        finally:
            set_world_paused(args.world, False)
            world_paused = False
        gazebo_held_pose = verify_fixture_height(args.model, pose.z)
        c.set_realized_spawn_height(gazebo_held_pose[2])
        wait_for_airborne_estimator_settle(c, pose.z)
        c.initialize_airborne_target()
        c.setup_logger()
        write_metadata(
            "spawn_config.json",
            {
                "requested": pose_dict,
                "controller": c.metadata(),
                "gazebo_held_pose": gazebo_held_pose,
            },
        )

        print(f"[runner] Pre-streaming neutral setpoints for {PRESTREAM_SECONDS:.1f}s...")
        stream_for(c, PRESTREAM_SECONDS)

        print("[runner] Requesting OFFBOARD at held airborne pose...")
        request_mode(c, "OFFBOARD")
        offboard = wait_heartbeat_state(
            c,
            MODE_TIMEOUT,
            predicate=lambda main, sub, armed: main == PX4_MAIN_MODE_OFFBOARD,
        )
        main_mode, sub_mode, armed = heartbeat_snapshot(c)
        if not offboard:
            raise RuntimeError(
                f"PX4 did not enter OFFBOARD "
                f"(main_mode={main_mode}, sub_mode={sub_mode}, armed={armed})"
            )
        print("[runner] OFFBOARD confirmed")

        print("[runner] Requesting arm at held airborne pose...")
        request_arm(c, True)
        armed_ok = wait_heartbeat_state(
            c,
            ARM_TIMEOUT,
            predicate=lambda main, sub, armed: armed is True,
        )
        main_mode, sub_mode, armed = heartbeat_snapshot(c)
        if not armed_ok:
            raise RuntimeError(
                f"PX4 did not arm "
                f"(main_mode={main_mode}, sub_mode={sub_mode}, armed={armed})"
            )
        print("[runner] Armed confirmed")
        print("[runner] Spooling propulsion to measured release readiness...")
        release_spool = spool_propulsion_for_release(c)

        # Detach while the last upward spool command is still current, then
        # immediately capture the free-flight hold target. This avoids both a
        # motor-idle drop and prolonged controller loading against the joint.
        detach_spawn_fixture()
        fixture_detached = True
        c.notify_fixture_released()
        write_metadata(
            "spawn_config.json",
            {
                "requested": pose_dict,
                "controller": c.metadata(),
                "gazebo_held_pose": gazebo_held_pose,
                "release_spool": release_spool,
            },
        )

        print("[runner] Starting randomized-initialization controller...")
        worker = threading.Thread(target=c.run, daemon=True)
        worker.start()

        deadline = time.monotonic() + MISSION_TIMEOUT
        while time.monotonic() < deadline and worker.is_alive():
            if c.phase == "DONE":
                completed = True
                print("[runner] Controlled approach reached landing handoff")
                break
            time.sleep(0.1)

        if not completed:
            main_mode, sub_mode, armed = heartbeat_snapshot(c)
            raise RuntimeError(
                "Mission did not complete "
                f"(phase={c.phase}, main_mode={main_mode}, "
                f"sub_mode={sub_mode}, armed={armed}, worker_error={c.worker_error})"
            )

        print("[runner] Handing final touchdown to PX4 LAND...")
        request_mode(c, "LAND")
        land_mode = wait_heartbeat_state(
            c,
            MODE_TIMEOUT,
            predicate=lambda main, sub, armed: (
                main == PX4_MAIN_MODE_AUTO and sub == PX4_AUTO_SUB_MODE_LAND
            ),
            keep_streaming=False,
        )
        main_mode, sub_mode, armed = heartbeat_snapshot(c)
        if not land_mode:
            raise RuntimeError(
                f"PX4 did not enter LAND "
                f"(main_mode={main_mode}, sub_mode={sub_mode}, armed={armed})"
            )
        disarmed = wait_heartbeat_state(
            c,
            LAND_TIMEOUT,
            predicate=lambda main, sub, armed: armed is False,
            keep_streaming=False,
        )
        main_mode, sub_mode, armed = heartbeat_snapshot(c)
        if not disarmed:
            raise RuntimeError(
                f"PX4 LAND did not auto-disarm within {LAND_TIMEOUT:.0f}s "
                f"(main_mode={main_mode}, sub_mode={sub_mode}, armed={armed})"
            )
        print("[runner] PX4 native landing/disarm confirmed")

        c.stop()
        controller_stopped = True
        if worker is not None:
            worker.join(timeout=3.0)
        print(
            f"[runner] SUCCESS seed={pose.seed} "
            f"spawn=({pose.x:.3f},{pose.y:.3f},{pose.z:.3f})"
        )

    finally:
        if world_paused:
            try:
                set_world_paused(args.world, False)
                world_paused = False
            except Exception as exc:
                print(f"[runner] Gazebo resume cleanup error: {exc!r}")
        neutral_stop.set()
        if neutral_thread is not None and neutral_thread.is_alive():
            neutral_thread.join(timeout=1.0)
        try:
            main_mode, sub_mode, armed = heartbeat_snapshot(c)
            if armed:
                if fixture_created and not fixture_detached:
                    print("[runner] Failure cleanup: detaching spawn fixture")
                    detach_spawn_fixture()
                    fixture_detached = True
                print(
                    "[runner] Failure cleanup: requesting PX4 LAND "
                    f"(phase={c.phase}, main_mode={main_mode}, sub_mode={sub_mode})"
                )
                request_mode(c, "LAND")
                wait_heartbeat_state(
                    c,
                    LAND_TIMEOUT,
                    predicate=lambda main, sub, armed: armed is False,
                    keep_streaming=False,
                )
        except Exception as exc:
            print(f"[runner] Failure LAND cleanup error: {exc!r}")
        finally:
            try:
                if c.master is not None and not controller_stopped:
                    c.stop()
            except Exception as exc:
                print(f"[runner] Controller cleanup error: {exc!r}")


if __name__ == "__main__":
    main()
