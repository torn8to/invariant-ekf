from state import RobotState, ImuUpdate, OdomUpdate
from offline_bag_read import ImuOdometryBagReader, TimeAlignedBagFeeder, StampedMsg
from imu_bias_ekf import InertialIEKF
from typing import Optional
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import typer
from typing import Annotated
from tqdm import tqdm


def convergence_check(
        data_run: Annotated[int, typer.Option(help="an integer for each data run",min=0, max=1)] = 0):
    run: dict[int, str] = {0: "bags/rosbag2_2026_02_18-22_55_56",
                        1:  "bags/rosbag2_2026_02_18-22_55_56"}
    bag_reader = ImuOdometryBagReader(run[data_run],
                                  imu_topic="/imu/data",
                                  odom_topic="/lid_odom")
    error_list: list[float] = []
    taf = TimeAlignedBagFeeder(bag_reader, 0.012465)
    ekf = InertialIEKF()
    odom_time: Optional[float] = None
    odom_msg: Optional[StampedMsg] = None

    with tqdm(taf) as pbar:
        for msg in pbar:
            msg_type = msg.msg_type
            if msg_type == Odometry:
                odom_msg = msg
                odom_time = msg.time
            elif msg_type == Imu:
                imu_update: ImuUpdate = ImuUpdate.fromMsg(msg.msg)
                if odom_time is not None and odom_time < msg.time:
                    odom_update: OdomUpdate = OdomUpdate.fromMsg(odom_msg.msg)
                    prediction_position = ekf.position
                    ekf.update(odom_update)
                    update_position = ekf.position
                    error.append(np.linalg.norm((predition_position - update_position)))

                    ekf.predict(imu_update)
                    odom_msg = None
                    odom_time = None
                else:
                    ekf.predict(imu_update)
        plot_error(error_list)


def plot_error(error: list[int]):
    iterations = range(len(error))
    fig, ax = plt.subplots()
    ax.plot(iterations, error)
    ax.sety_label("Error (M)")
    ax.setx_label("Updates")
    ax.set_title("Error upon each EKF update")




if __name__ == "__main__":
    convergence_check()
