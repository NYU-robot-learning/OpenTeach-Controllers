from typing import Tuple
import numpy as np

from deoxys.utils import transform_utils

def _min_jerk_spaces(N: int, T: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Generates a 1-dim minimum jerk trajectory from 0 to 1 in N steps & T seconds.
    Assumes zero velocity & acceleration at start & goal.
    The resulting trajectories can be scaled for different start & goals.

    Args:
        N: Length of resulting trajectory in steps
        T: Duration of resulting trajectory in seconds

    Returns:
        p_traj: Position trajectory of shape (N,)
        pd_traj: Velocity trajectory of shape (N,)
        pdd_traj: Acceleration trajectory of shape (N,)
    """
    assert N > 1, "Number of planning steps must be larger than 1."

    t_traj = np.linspace(0, 1, N)
    p_traj = 10 * t_traj**3 - 15 * t_traj**4 + 6 * t_traj**5
    pd_traj = (30 * t_traj**2 - 60 * t_traj**3 + 30 * t_traj**4) / T
    pdd_traj = (60 * t_traj - 180 * t_traj**2 + 120 * t_traj**3) / (T**2)

    return p_traj, pd_traj, pdd_traj


def generate_cartesian_space_min_jerk(
    start: np.ndarray,
    goal: np.ndarray,
    hz: float,
    time_to_go: float = 5.,
) -> np.ndarray:
    """Initializes planner object and plans the trajectory

    Args:
        start: Start pose
        goal: Goal pose
        time_to_go: Trajectory duration in seconds
        hz: Frequency of output trajectory

    Returns:
        q_traj: Joint position trajectory
        qd_traj: Joint velocity trajectory
        qdd_traj: Joint acceleration trajectory
    """
    steps = int(time_to_go * hz + 1)
    dt = 1.0 / hz

    p_traj, pd_traj, pdd_traj = _min_jerk_spaces(steps, time_to_go)

    x_start = start[:3]
    x_goal = goal[:3]

    # Plan translation

    D = x_goal - x_start
    x_traj = x_start[None, :] + D[None, :] * p_traj[:, None]
    # xd_traj = D[None, :] * pd_traj[:, None]
    # xdd_traj = D[None, :] * pdd_traj[:, None]

    # Plan rotation
    r_start = start[3:]
    r_goal = goal[3:]
    r_delta = transform_utils.quat_multiply(r_goal, transform_utils.quat_inverse(r_start))
    rv_delta = transform_utils.quat2axisangle(r_delta)

    # r_start = start.rotation()
    # r_goal = goal.rotation()
    # r_delta = r_goal * r_start.inv()
    # rv_delta = r_delta.as_rotvec()

    r_traj = np.zeros([steps, 4])
    for i in range(steps):
        r_traj[i, :] = transform_utils.quat_multiply(
            transform_utils.axisangle2quat(rv_delta * p_traj[i]),
            r_start
        )

    # rd_traj = rv_delta[None, :] * pd_traj[:, None]
    # rdd_traj = rv_delta[None, :] * pdd_traj[:, None]

    # return (
    #     np.concatenate([x_traj, r_traj], axis=1),
    #     np.concatenate([xd_traj, rd_traj], axis=1),
    #     np.concatenate([xdd_traj, rdd_traj], axis=1),
    # )

    return np.concatenate([x_traj, r_traj], axis=1)