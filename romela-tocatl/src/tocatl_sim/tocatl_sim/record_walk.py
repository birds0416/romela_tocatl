"""Reproducible offscreen recorder for the TOCATL walk (no ROS needed).

Runs physics + the ported gait in-process, renders with mujoco.Renderer and
writes an .mp4 via OpenCV. The default script is the camera pass from the sim
brief: stand, smooth tripod walk forward, a gentle turn, stop to a clean stand.

    ros2 run tocatl_sim record_walk -- --out walk.mp4 --camera track34
    python3 -m tocatl_sim.record_walk --gait ripple --seconds 20

Set MUJOCO_GL=egl (or osmesa) if running without a display.
"""

from __future__ import annotations

import argparse

import numpy as np

import mujoco

from .gait_engine import GaitEngine
from .hexapod_sim import HexapodSim

CTRL_DT = 0.005  # 200 Hz, the real command-stream rate


def default_script(total: float):
    """(t_start, t_end, (vx, vy, yaw)) segments, normalized commands."""
    return [
        (0.0, 1.5, (0.0, 0.0, 0.0)),          # settle into the stand
        (1.5, total * 0.55, (1.0, 0.0, 0.0)),  # smooth walk forward
        (total * 0.55, total * 0.85, (0.7, 0.0, 0.45)),  # gentle arcing turn
        (total * 0.85, total - 1.0, (0.0, 0.0, 0.0)),    # stop -> stand
    ]


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="walk.mp4")
    parser.add_argument("--seconds", type=float, default=18.0)
    parser.add_argument("--fps", type=int, default=60)
    parser.add_argument("--camera", default="track34",
                        help="track34 | front | fixed34")
    parser.add_argument("--gait", default="tripod", help="tripod | ripple | wave")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args(argv)

    sim = HexapodSim()
    eng = GaitEngine(gait_name=args.gait)
    steps_per_ctrl = round(CTRL_DT / sim.timestep)
    renderer = mujoco.Renderer(sim.model, height=args.height, width=args.width)

    import cv2
    writer = cv2.VideoWriter(
        args.out, cv2.VideoWriter_fourcc(*"mp4v"), args.fps,
        (args.width, args.height))
    if not writer.isOpened():
        raise RuntimeError(f"could not open video writer for {args.out}")

    script = default_script(args.seconds)
    n_ctrl = int(args.seconds / CTRL_DT)
    frame_period = 1.0 / args.fps
    next_frame = 0.0
    frames = 0
    stopped = False

    for i in range(n_ctrl):
        t = i * CTRL_DT
        cmd = (0.0, 0.0, 0.0)
        for t0, t1, c in script:
            if t0 <= t < t1:
                cmd = c
                break
        if cmd == (0.0, 0.0, 0.0) and t > args.seconds * 0.8 and not stopped:
            eng.stop()  # graceful return for the final stand
            stopped = True
        eng.set_velocity(*cmd)
        c = eng.update(CTRL_DT)
        sim.set_command(c.position, c.velocity, c.kp, c.kd, c.tau_ff)
        sim.step(steps_per_ctrl)

        if t >= next_frame:
            renderer.update_scene(sim.data, camera=args.camera)
            rgb = renderer.render()
            writer.write(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            frames += 1
            next_frame += frame_period

    writer.release()
    base, _ = sim.base_pose()
    print(f"wrote {args.out}: {frames} frames @ {args.fps} fps, "
          f"{args.seconds:.1f} s sim, final base ({base[0]:+.2f}, {base[1]:+.2f}, {base[2]:.2f}) m, "
          f"NaNs: {bool(np.isnan(sim.data.qpos).any())}")


if __name__ == "__main__":
    main()
