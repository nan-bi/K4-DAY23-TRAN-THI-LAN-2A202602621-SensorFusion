"""Bonus analysis script for Day 23 Sensor Fusion.

Performs:
1. Calibration sensitivity analysis with multiple extrinsic perturbation levels.
2. BEV trajectory and camera projection visualization.
3. CVAT track export.
"""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import matplotlib.pyplot as plt
import numpy as np

from fusion_lab import tracking_params as params
from fusion_lab.export_cvat import export_tracks_json
from fusion_lab.tracking.filter import Filter
from fusion_lab.tracking.manager import Track, TrackManager
from fusion_lab.tracking.sensors import Measurement, Sensor
from fusion_lab.workspace_loader import load_workspace


def run_bonus_analysis() -> None:
    modules = load_workspace()
    kalman = modules["kalman"]
    association = modules["association"]
    camera_fusion = modules["camera_fusion"]
    track_mgmt = modules["track_management"]

    output_dir = Path(__file__).resolve().parent
    output_dir.mkdir(parents=True, exist_ok=True)

    # -------------------------------------------------------------
    # 1. Setup camera and vehicle simulation scenario
    # -------------------------------------------------------------
    # True camera calibration
    calib = SimpleNamespace(
        intrinsic=[1000.0, 1000.0, 960.0, 640.0],
        width=1920.0,
        height=1280.0,
        extrinsic=SimpleNamespace(transform=np.eye(4, dtype=float).ravel().tolist()),
    )
    lidar_sensor = Sensor("lidar", None, camera_fusion)
    true_camera = Sensor("camera", calib, camera_fusion)

    # Simulated trajectory: Vehicle moving along curve
    num_frames = 50
    dt = params.dt
    np.random.seed(42)

    gt_states = []
    curr_pos = np.array([12.0, -1.5, 0.0])
    curr_vel = np.array([10.0, 0.8, 0.0])
    for f in range(num_frames):
        gt_states.append(curr_pos.copy())
        curr_pos = curr_pos + curr_vel * dt + np.array([0.0, 0.05 * np.sin(f * 0.2), 0.0])

    # Generate LiDAR & Camera measurements
    lidar_measurements = []
    cam_measurements = []

    for f, pos in enumerate(gt_states):
        # LiDAR noisy measurement [x, y, z, h, w, l, yaw]
        noisy_lidar_pos = pos + np.random.normal(0, params.sigma_lidar_x, size=3)
        lidar_measurements.append(
            Measurement(f, [noisy_lidar_pos[0], noisy_lidar_pos[1], noisy_lidar_pos[2], 1.6, 2.0, 4.2, 0.05], lidar_sensor)
        )
        # Camera noisy measurement [u, v]
        state_vec = np.asmatrix(np.r_[pos, [0, 0, 0]]).T
        if true_camera.in_fov(state_vec):
            true_px = true_camera.get_hx(state_vec)
            noisy_px = [
                float(true_px[0, 0]) + np.random.normal(0, params.sigma_cam_i),
                float(true_px[1, 0]) + np.random.normal(0, params.sigma_cam_j),
            ]
            cam_measurements.append(
                Measurement(f, noisy_px, true_camera, R=np.asmatrix(np.diag([params.sigma_cam_i**2, params.sigma_cam_j**2])))
            )
        else:
            cam_measurements.append(None)

    # -------------------------------------------------------------
    # 2. Calibration Sensitivity Evaluation
    # -------------------------------------------------------------
    perturbations = [
        ("Baseline (LiDAR Only)", 0.0, 0.0, False),
        ("Calibrated Fusion (0.0° / 0.0m)", 0.0, 0.0, True),
        ("Yaw Offset +1.0° (+0.017 rad)", 0.0175, 0.0, True),
        ("Yaw Offset +2.5° (+0.044 rad)", 0.0436, 0.0, True),
        ("Yaw Offset +5.0° (+0.087 rad)", 0.0873, 0.0, True),
        ("Lateral Offset dy = +0.2 m", 0.0, 0.2, True),
        ("Lateral Offset dy = +0.5 m", 0.0, 0.5, True),
        ("Lateral Offset dy = +1.0 m", 0.0, 1.0, True),
    ]

    results_table = []
    trajectories = {}

    for name, yaw_err, dy_err, use_camera in perturbations:
        # Create perturbed camera
        pert_transform = np.eye(4, dtype=float)
        cos_y, sin_y = np.cos(yaw_err), np.sin(yaw_err)
        pert_transform[0, 0] = cos_y
        pert_transform[0, 1] = -sin_y
        pert_transform[1, 0] = sin_y
        pert_transform[1, 1] = cos_y
        pert_transform[1, 3] = dy_err

        pert_calib = SimpleNamespace(
            intrinsic=[1000.0, 1000.0, 960.0, 640.0],
            width=1920.0,
            height=1280.0,
            extrinsic=SimpleNamespace(transform=pert_transform.ravel().tolist()),
        )
        test_camera = Sensor("camera", pert_calib, camera_fusion)

        manager = TrackManager(track_mgmt)
        filt = Filter(kalman)

        innovations = []
        gating_rejections = 0
        total_cam_attempts = 0
        sq_errors = []
        track_traj = []

        for f in range(num_frames):
            # 1. EKF Predict
            for track in manager.track_list:
                filt.predict(track)

            # 2. AssocL
            l_meas = [lidar_measurements[f]]
            association.associate_and_update(manager, l_meas, filt, lidar_sensor)

            # 3. AssocC (if enabled)
            if use_camera and cam_measurements[f] is not None:
                # Use test_camera (with calibration perturbation) as the tracker's camera model
                c_meas = [Measurement(f, cam_measurements[f].z, test_camera, R=cam_measurements[f].R)]
                # Evaluate innovation and gating before update
                for track in manager.track_list:
                    if test_camera.in_fov(track.x):
                        total_cam_attempts += 1
                        H = test_camera.get_H(track.x)
                        gamma = kalman.innovation(track.x, c_meas[0])
                        S = kalman.innovation_covariance(track.P, c_meas[0], H)
                        mhd = float(np.asarray(gamma.T @ np.linalg.inv(S) @ gamma).item())
                        innovations.append(float(np.linalg.norm(gamma)))
                        if not association.chi2_gate(mhd, test_camera):
                            gating_rejections += 1
                association.associate_and_update(manager, c_meas, filt, test_camera)

            # Record estimated position
            if len(manager.track_list) > 0:
                est_p = np.asarray(manager.track_list[0].x)[:3].ravel()
                track_traj.append(est_p.copy())
                sq_errors.append(np.sum((est_p - gt_states[f]) ** 2))
            else:
                track_traj.append(None)

        rmse = np.sqrt(np.mean(sq_errors)) if sq_errors else float("nan")
        mean_innov = np.mean(innovations) if innovations else 0.0
        rejection_rate = (gating_rejections / total_cam_attempts * 100.0) if total_cam_attempts > 0 else 0.0

        results_table.append({
            "name": name,
            "rmse_m": float(rmse),
            "mean_innov_px": float(mean_innov),
            "rejection_rate_pct": float(rejection_rate),
        })
        trajectories[name] = track_traj

    # -------------------------------------------------------------
    # 3. Save Summary Table JSON
    # -------------------------------------------------------------
    table_path = output_dir / "calibration_results.json"
    table_path.write_text(json.dumps(results_table, indent=2), encoding="utf-8")

    # -------------------------------------------------------------
    # 4. Generate Visualization Figures
    # -------------------------------------------------------------
    # Figure 1: BEV Trajectory comparison
    fig1, ax1 = plt.subplots(figsize=(10, 6), dpi=150)
    gt_arr = np.array(gt_states)
    ax1.plot(gt_arr[:, 0], gt_arr[:, 1], "k-", linewidth=2.5, label="Ground Truth")

    lidar_traj = [p for p in trajectories["Baseline (LiDAR Only)"] if p is not None]
    if lidar_traj:
        l_arr = np.array(lidar_traj)
        ax1.plot(l_arr[:, 0], l_arr[:, 1], "b--o", markersize=4, label=f"LiDAR Only (RMSE={results_table[0]['rmse_m']:.3f}m)")

    fused_traj = [p for p in trajectories["Calibrated Fusion (0.0° / 0.0m)"] if p is not None]
    if fused_traj:
        f_arr = np.array(fused_traj)
        ax1.plot(f_arr[:, 0], f_arr[:, 1], "g-^", markersize=4, label=f"Fused Clean (RMSE={results_table[1]['rmse_m']:.3f}m)")

    pert_traj = [p for p in trajectories["Yaw Offset +2.5° (+0.044 rad)"] if p is not None]
    if pert_traj:
        p_arr = np.array(pert_traj)
        ax1.plot(p_arr[:, 0], p_arr[:, 1], "r-s", markersize=4, label=f"Fused +2.5° Miscalibrated (RMSE={results_table[3]['rmse_m']:.3f}m)")

    ax1.set_xlabel("X [Forward, m]", fontsize=12)
    ax1.set_ylabel("Y [Lateral, m]", fontsize=12)
    ax1.set_title("BEV Vehicle Tracking Trajectory Comparison under Calibration Offsets", fontsize=13, fontweight="bold")
    ax1.grid(True, linestyle="--", alpha=0.6)
    ax1.legend(loc="best", fontsize=10)
    plt.tight_layout()
    fig1.savefig(output_dir / "bev_tracking_comparison.png")
    plt.close(fig1)

    # Figure 2: Calibration Sensitivity Curves (RMSE & Gating Rejection)
    yaw_angles_deg = [0.0, 1.0, 2.5, 5.0]
    yaw_rmse = [results_table[i]["rmse_m"] for i in [1, 2, 3, 4]]
    yaw_innov = [results_table[i]["mean_innov_px"] for i in [1, 2, 3, 4]]
    yaw_rej = [results_table[i]["rejection_rate_pct"] for i in [1, 2, 3, 4]]

    fig2, (ax2_1, ax2_2) = plt.subplots(1, 2, figsize=(13, 5), dpi=150)
    ax2_1.plot(yaw_angles_deg, yaw_rmse, "r-o", linewidth=2, label="Tracking RMSE (m)")
    ax2_1.axhline(results_table[0]["rmse_m"], color="b", linestyle="--", label="LiDAR Only Baseline")
    ax2_1.set_xlabel("Camera Extrinsic Yaw Offset [degrees]", fontsize=11)
    ax2_1.set_ylabel("3D Position RMSE [m]", fontsize=11)
    ax2_1.set_title("RMSE vs Extrinsic Angular Miscalibration", fontsize=12, fontweight="bold")
    ax2_1.grid(True, linestyle="--", alpha=0.6)
    ax2_1.legend(fontsize=10)

    ax2_2.plot(yaw_angles_deg, yaw_innov, "m-s", linewidth=2, label="Mean Innovation Residual [px]")
    ax2_2.plot(yaw_angles_deg, yaw_rej, "c-^", linewidth=2, label="Chi² Gating Rejection [%]")
    ax2_2.set_xlabel("Camera Extrinsic Yaw Offset [degrees]", fontsize=11)
    ax2_2.set_ylabel("Pixel Residual / Rejection Rate (%)", fontsize=11)
    ax2_2.set_title("Innovation & Chi² Gate Rejection vs Miscalibration", fontsize=12, fontweight="bold")
    ax2_2.grid(True, linestyle="--", alpha=0.6)
    ax2_2.legend(fontsize=10)

    plt.tight_layout()
    fig2.savefig(output_dir / "calibration_sensitivity_curve.png")
    plt.close(fig2)

    # Figure 3: Camera Image Plane 2D Projection and Innovation Vector
    fig3, ax3 = plt.subplots(figsize=(10, 6), dpi=150)
    ax3.set_xlim(600, 1300)
    ax3.set_ylim(800, 300)  # Inverted Y for image plane

    pred_pixels = []
    meas_pixels = []
    for f in range(20):
        pos = gt_states[f]
        state = np.asmatrix(np.r_[pos, [0, 0, 0]]).T
        pred_p = true_camera.get_hx(state)
        meas_p = cam_measurements[f].z if cam_measurements[f] is not None else pred_p
        pred_pixels.append([float(pred_p[0, 0]), float(pred_p[1, 0])])
        meas_pixels.append([float(meas_p[0, 0]), float(meas_p[1, 0])])

    pred_arr = np.array(pred_pixels)
    meas_arr = np.array(meas_pixels)

    ax3.plot(pred_arr[:, 0], pred_arr[:, 1], "b--o", label="EKF Predicted Pixel Projection h(x)")
    ax3.scatter(meas_arr[:, 0], meas_arr[:, 1], c="r", marker="x", s=60, label="Noisy Camera Measurement z")
    for i in range(0, len(pred_arr), 3):
        ax3.annotate("", xy=(meas_arr[i, 0], meas_arr[i, 1]), xytext=(pred_arr[i, 0], pred_arr[i, 1]),
                     arrowprops=dict(arrowstyle="->", color="purple", lw=1.5))

    ax3.set_xlabel("Image U coordinate [pixels]", fontsize=12)
    ax3.set_ylabel("Image V coordinate [pixels]", fontsize=12)
    ax3.set_title("Camera Image Plane 2D Projection, Measurements & Innovation Vectors", fontsize=13, fontweight="bold")
    ax3.grid(True, linestyle="--", alpha=0.6)
    ax3.legend(loc="best", fontsize=10)
    plt.tight_layout()
    fig3.savefig(output_dir / "camera_projection_and_update.png")
    plt.close(fig3)

    # -------------------------------------------------------------
    # 5. Export CVAT Format Tracking Output
    # -------------------------------------------------------------
    cvat_frames = []
    for f in range(num_frames):
        frame_tracks = []
        if trajectories["Calibrated Fusion (0.0° / 0.0m)"][f] is not None:
            p = trajectories["Calibrated Fusion (0.0° / 0.0m)"][f]
            frame_tracks.append({
                "id": 1,
                "box": [float(p[0]), float(p[1]), float(p[2]), 1.6, 2.0, 4.2],
                "score": 1.0,
                "state": "confirmed",
            })
        cvat_frames.append({"frame": f, "tracks": frame_tracks})

    cvat_export_file = output_dir / "cvat_tracks_sample.json"
    export_tracks_json(cvat_frames, cvat_export_file)

    print("Bonus analysis completed successfully!")
    for item in results_table:
        print(f"{item['name']:35s} | RMSE: {item['rmse_m']:.4f}m | Mean Innov: {item['mean_innov_px']:.2f}px | Rejection: {item['rejection_rate_pct']:.1f}%")


if __name__ == "__main__":
    run_bonus_analysis()
