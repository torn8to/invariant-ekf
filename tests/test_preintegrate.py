import unittest
import torch
import pypose as pp
from pypose.module.imu_preintegrator import IMUPreintegrator
from imu_bias_ekf import ImuUpdate

def _normalize_quat_xyzw(q: torch.Tensor) -> torch.Tensor:
    return q / (q.norm(dim=-1, keepdim=True).clamp_min(1e-12))


def _align_quat_sign(ref_q: torch.Tensor, q: torch.Tensor) -> torch.Tensor:
    dot = (ref_q * q).sum(dim=-1, keepdim=True)
    sign = torch.where(dot < 0, -torch.ones_like(dot), torch.ones_like(dot))
    return q * sign


def generate_imu_data(
    *,
    num_steps: int = 50,
    dt_min: float = 0.002,
    dt_max: float = 0.02,
    accel_std: float = 1.0,
    gyro_std: float = 0.2,
    seed: int = 0,
    dtype: torch.dtype = torch.float64,
    device: torch.device | str = "cpu",
) -> dict[str, torch.Tensor]:
    """
    Generate random IMU stream as tensors:
      - dt: (F, 1)
      - gyro: (F, 3)
      - acc: (F, 3)
      - quat: (F, 4) in (x, y, z, w)
      - timestamps: (F+1,) cumulative, starting at 0
    """
    if num_steps <= 0:
        raise ValueError("num_steps must be > 0")

    g = torch.Generator(device="cpu")
    g.manual_seed(seed)

    dt = (dt_min + (dt_max - dt_min) * torch.rand((num_steps, 1), generator=g, dtype=dtype)).to(device)
    timestamps = torch.zeros((num_steps + 1,), dtype=dtype, device=device)
    timestamps[1:] = torch.cumsum(dt.squeeze(-1), dim=0)

    gyro = (gyro_std * torch.randn((num_steps, 3), generator=g, dtype=dtype)).to(device)
    acc = (accel_std * torch.randn((num_steps, 3), generator=g, dtype=dtype)).to(device)

    # Random quaternion stream (not necessarily dynamically consistent, but valid unit quats)
    quat = torch.randn((num_steps, 4), generator=g, dtype=dtype).to(device)
    quat = _normalize_quat_xyzw(quat)

    return {"dt": dt, "timestamps": timestamps, "gyro": gyro, "acc": acc, "quat": quat}


def imu_tensors_to_updates(
    *,
    timestamps: torch.Tensor,
    acc: torch.Tensor,
    gyro: torch.Tensor,
    quat: torch.Tensor,
) -> list[ImuUpdate]:
    if timestamps.ndim != 1:
        raise ValueError("timestamps must be 1D (F+1,)")
    if acc.shape[-1] != 3 or gyro.shape[-1] != 3 or quat.shape[-1] != 4:
        raise ValueError("acc/gyro must end with 3 and quat must end with 4")
    if timestamps.numel() != acc.shape[0] + 1:
        raise ValueError("timestamps must have length F+1")
    if acc.shape[0] != gyro.shape[0] or acc.shape[0] != quat.shape[0]:
        raise ValueError("acc/gyro/quat must have matching leading dimension F")

    ts = timestamps.detach().cpu().tolist()
    updates: list[ImuUpdate] = []
    for k in range(acc.shape[0]):
        updates.append(
            ImuUpdate(
                timestamp=float(ts[k + 1]),
                accelerometer=acc[k].detach().cpu().numpy(),
                angular_velocity=gyro[k].detach().cpu().numpy(),
                orientation=quat[k].detach().cpu().numpy(),
            )
        )
    return updates

class PreintegrationTest(unittest.TestCase):
    def test_imu_preintegrator_sequence_matches_stepwise(self):
        # Keep everything in float32 to match IMUPreintegrator's internal buffers (e.g. gravity).
        data = generate_imu_data(num_steps=100, seed=123, dtype=torch.float32)
        dt, gyro, acc, quat = data["dt"], data["gyro"], data["acc"], data["quat"]

        rot = pp.SO3(quat)  # (F, 4) -> SO3 LieTensor

        # One-shot integration over the full sequence (returns (1, F, ...))
        integrator_seq = IMUPreintegrator(
            pos=torch.zeros(3, dtype=dt.dtype),
            rot=pp.identity_SO3(dtype=dt.dtype),
            vel=torch.zeros(3, dtype=dt.dtype),
            prop_cov=True,
            reset=True,
        )
        states_seq = integrator_seq(dt, gyro, acc, rot)

        # Step-wise integration, accumulating outputs across calls.
        integrator_step = IMUPreintegrator(
            pos=torch.zeros(3, dtype=dt.dtype),
            rot=pp.identity_SO3(dtype=dt.dtype),
            vel=torch.zeros(3, dtype=dt.dtype),
            prop_cov=True,
            reset=False,
        )
        step_pos, step_vel, step_rot = [], [], []
        for k in range(dt.shape[0]):
            out = integrator_step(dt[k], gyro[k], acc[k], rot[k])
            step_pos.append(out["pos"])
            step_vel.append(out["vel"])
            step_rot.append(out["rot"])

        pos_step = torch.cat(step_pos, dim=1)
        vel_step = torch.cat(step_vel, dim=1)
        rot_step = pp.SO3(torch.cat([r.tensor() for r in step_rot], dim=1))

        torch.testing.assert_close(pos_step, states_seq["pos"], rtol=1e-5, atol=1e-6)
        torch.testing.assert_close(vel_step, states_seq["vel"], rtol=1e-5, atol=1e-6)

        q_seq = states_seq["rot"].tensor()
        q_step = rot_step.tensor()
        q_step = _align_quat_sign(q_seq, q_step)
        torch.testing.assert_close(q_step, q_seq, rtol=1e-5, atol=1e-6)

    def test_generate_imu_data_and_convert_to_updates(self):
        data = generate_imu_data(num_steps=10, seed=0, dtype=torch.float64)
        updates = imu_tensors_to_updates(
            timestamps=data["timestamps"],
            acc=data["acc"],
            gyro=data["gyro"],
            quat=data["quat"],
        )
        self.assertEqual(len(updates), 10)
        self.assertGreater(updates[-1].timestamp, updates[0].timestamp)
        self.assertEqual(updates[0].accelerometer.shape, (3,))
        self.assertEqual(updates[0].angular_velocity.shape, (3,))
        self.assertEqual(updates[0].orientation.shape, (4,))

