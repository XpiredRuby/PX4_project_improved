"""Register an aircraft profile against inspected local Gazebo/PX4 model files.

This is an operator-side CLI, never a browser-supplied path or command. A
binding fingerprints simulator assets and checks basic mass/rotor consistency;
it is not physical identification or controller-tuning evidence.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from .aircraft import assess_profile
from .contracts import canonical_sha256, load_contract


def fingerprint(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def inspect_model(autopilot, model):
    if not isinstance(model, str) or not re.fullmatch(r"[a-zA-Z0-9_]+", model):
        raise ValueError("invalid Gazebo model name")
    autopilot = Path(autopilot).resolve()
    models = autopilot / "Tools/simulation/gz/models"
    links, joints, motors, files, visiting = {}, {}, [], {}, set()

    def visit(name):
        if name in visiting:
            raise ValueError("recursive/duplicate model inclusion is unsupported")
        visiting.add(name)
        folder = (models / name).resolve()
        if not folder.is_relative_to(models.resolve()):
            raise ValueError("model path must stay within the PX4 model directory")
        root = ET.parse(folder / "model.sdf").getroot().find("model")
        if root is None:
            raise ValueError("expected one SDF model")
        for path in sorted(folder.rglob("*")):
            if path.is_file():
                if not path.resolve().is_relative_to(models.resolve()):
                    raise ValueError("external model asset symlinks are unsupported")
                files[str(path.relative_to(autopilot))] = fingerprint(path)
        for include in root.findall("include"):
            uri = include.findtext("uri", "")
            pose = [float(x) for x in include.findtext("pose", "0 0 0 0 0 0").split()]
            if (not re.fullmatch(r"model://[a-zA-Z0-9_]+", uri)
                    or include.get("merge") != "true" or any(pose)):
                raise ValueError("only local merge=true model includes with identity pose are supported")
            visit(uri.removeprefix("model://"))
        for link in root.findall("link"):
            name = link.get("name")
            if name in links:
                raise ValueError("duplicate link name in merged model")
            pose = link.find("pose")
            if pose is not None and pose.get("relative_to"):
                raise ValueError("relative link pose requires a transform-aware model adapter")
            links[name] = link
        for joint in root.findall("joint"):
            if joint.get("name") in joints:
                raise ValueError("duplicate joint in merged model")
            joints[joint.get("name")] = joint
        motors.extend(p for p in root.findall("plugin") if "MulticopterMotorModel" in p.get("name", ""))

    visit(model)
    try:
        motors.sort(key=lambda p: int(p.findtext("motorNumber")))
        numbers = [int(p.findtext("motorNumber")) for p in motors]
    except (TypeError, ValueError) as exc:
        raise ValueError("each motor needs a numeric motorNumber") from exc
    if numbers != list(range(len(motors))):
        raise ValueError("motor numbers must be unique and contiguous from zero")
    mass, yaw_inertia = 0., 0.
    rotors = []
    for link in links.values():
        m = float(link.findtext("inertial/mass", "0"))
        pose = [float(x) for x in link.findtext("pose", "0 0 0 0 0 0").split()]
        ipose = [float(x) for x in link.findtext("inertial/pose", "0 0 0 0 0 0").split()]
        if len(pose) != 6 or len(ipose) != 6 or any(pose[3:]) or any(ipose):
            raise ValueError("rotated/offset inertials require a transform-aware model adapter")
        mass += m
        yaw_inertia += float(link.findtext("inertial/inertia/izz", "0")) + m*(pose[0]**2+pose[1]**2)
    for motor in motors:
        link = links.get(motor.findtext("linkName"))
        if link is None:
            raise ValueError("motor references an unknown rotor link")
        joint = joints.get(motor.findtext("jointName"))
        if joint is None or joint.findtext("child") != link.get("name"):
            raise ValueError("motor joint must identify its rotor link")
        axis = joint.find("axis/xyz")
        joint_pose = [float(x) for x in joint.findtext("pose", "0 0 0 0 0 0").split()]
        if (axis is None or axis.get("expressed_in") or any(joint_pose)
                or [float(x) for x in axis.text.split()] != [0., 0., 1.]):
            raise ValueError("nonparallel/relative rotor axes need a different model adapter")
        pose = [float(x) for x in link.findtext("pose", "0 0 0 0 0 0").split()]
        direction = motor.findtext("turningDirection")
        if direction not in ("cw", "ccw"):
            raise ValueError("unknown rotor turning direction")
        coefficient = float(motor.findtext("momentConstant"))
        maximum = float(motor.findtext("motorConstant")) * float(motor.findtext("maxRotVelocity"))**2
        rotors.append(dict(x_m=pose[0], y_m=-pose[1],
                           yaw_torque_per_thrust_m=coefficient*(1 if direction == "ccw" else -1),
                           minimum_thrust_n=0., maximum_thrust_n=maximum))
    if mass <= 0 or not 4 <= len(rotors) <= 16:
        raise ValueError("expected a positive-mass parallel-axis 4–16 rotor model")
    return dict(model=model, mass_kg=mass, yaw_inertia_kg_m2=yaw_inertia,
                rotors=rotors, model_files_sha256=files)


def create_binding(profile, autopilot, model, airframe_id):
    setup = assess_profile(profile)
    if not setup["ready"]:
        raise ValueError("complete the aircraft profile before binding its simulator")
    inspection = inspect_model(autopilot, model)
    if len(inspection["rotors"]) > 12:
        raise ValueError("this PX4 adapter supports at most 12 rotors; the planner can represent up to 16")
    declared = setup["vehicle_model"]
    if abs(declared["mass_kg"] - inspection["mass_kg"]) > 1e-5:
        raise ValueError("profile mass does not match the total SDF link mass")
    if declared["yaw_inertia_kg_m2"] + 1e-8 < inspection["yaw_inertia_kg_m2"]:
        raise ValueError("profile yaw inertia is below the SDF inertia")
    remaining = list(inspection["rotors"])
    for rotor in declared["rotors"]:
        matches = [r for r in remaining if all(abs(r[k]-rotor[k]) < 1e-7 for k in
                   ("x_m", "y_m", "yaw_torque_per_thrust_m"))]
        if not matches or rotor["maximum_thrust_n"] > matches[0]["maximum_thrust_n"] + 1e-7:
            raise ValueError("profile rotor geometry, spin coefficient or available thrust does not match SDF")
        remaining.remove(matches[0])
    if remaining:
        raise ValueError("profile rotor count does not match SDF")
    if isinstance(airframe_id, bool) or not isinstance(airframe_id, int) or airframe_id <= 0:
        raise ValueError("airframe ID must be a positive integer")
    airframes = list((Path(autopilot) / "ROMFS/px4fmu_common/init.d-posix/airframes").glob(f"{airframe_id}_*"))
    if len(airframes) != 1:
        raise ValueError("airframe ID must resolve to one local PX4 startup script")
    airframe = airframes[0]
    inspection["model_files_sha256"][str(airframe.relative_to(autopilot))] = fingerprint(airframe)
    controller_parameters = {"CA_AIRFRAME": 0, "CA_ROTOR_COUNT": len(inspection["rotors"])}
    for i, rotor in enumerate(inspection["rotors"]):
        controller_parameters.update({f"CA_ROTOR{i}_PX": rotor["x_m"],
                                      f"CA_ROTOR{i}_PY": rotor["y_m"],
                                      f"CA_ROTOR{i}_KM": rotor["yaw_torque_per_thrust_m"],
                                      f"CA_ROTOR{i}_AX": 0., f"CA_ROTOR{i}_AY": 0., f"CA_ROTOR{i}_AZ": -1.,
                                      f"CA_ROTOR{i}_CT": rotor["maximum_thrust_n"] / max(r["maximum_thrust_n"] for r in inspection["rotors"])})
    return dict(schema_version=1, profile=setup["profile"], profile_sha256=setup["profile_sha256"],
                vehicle_model_sha256=setup["vehicle_model_sha256"], airframe_id=airframe_id,
                controller_parameters=controller_parameters, **inspection)


def validate_binding(binding, autopilot):
    if not isinstance(binding, dict) or binding.get("schema_version") != 1:
        raise ValueError("invalid simulation binding")
    expected = create_binding(binding["profile"], Path(autopilot).resolve(),
                              binding["model"], binding["airframe_id"])
    if canonical_sha256(expected) != canonical_sha256(binding):
        raise ValueError("simulation binding changed or simulator assets no longer match; register again")
    return expected


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--autopilot", type=Path, required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--airframe-id", type=int)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.profile:
        if args.airframe_id is None or args.output is None:
            parser.error("registration requires --airframe-id and --output")
        result = create_binding(load_contract(args.profile, dict), args.autopilot.resolve(), args.model, args.airframe_id)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
    else:
        result = inspect_model(args.autopilot, args.model)
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
