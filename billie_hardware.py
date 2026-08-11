import time
# import mujoco
# import mujoco.viewer
import numpy as np
import pinocchio as pin
import matplotlib.pyplot as plt
from cybergear import CyberGear
from scipy.interpolate import CubicSpline, CubicHermiteSpline

from Controllers.pd_controller import PDController
from Controllers.feedback_linearization_controller import FeedbackLinearization
from Controllers.linear_quadratic_regulizer_controller import TVLQR

################## 1. billie #####################

################## 2. Controller #####################
# (2.a) intitialize pinocchio biped model and load + interpolate reference trajectories
# urdf_path = r"C:\Users\julie\OneDrive - Georgia Institute of Technology\courses\Capstone\Code\Biped_Simulation\Controllers\URDF\biped_billie_linkage_pinned-original.urdf"
urdf_path = r"/home/bliss/biped_ws/Controllers/URDF/biped_billie_linkage_pinned-original.urdf"
biped = pin.buildModelFromUrdf(str(urdf_path))
pin_d = biped.createData()

# x_ref = np.load(r"C:\Users\julie\OneDrive - Georgia Institute of Technology\courses\Capstone\Code\Biped_Simulation\Data\Xsol_without-height.npy")
# u_ref = np.load(r"C:\Users\julie\OneDrive - Georgia Institute of Technology\courses\Capstone\Code\Biped_Simulation\Data\Usol_without-height.npy")
x_ref = np.load(r"/home/bliss/biped_ws/Xsol_without-height.npy")
u_ref = np.load(r"/home/bliss/biped_ws/Usol_without-height.npy")

x_ref = -x_ref
u_ref = -u_ref

step_duration = 0.2
dt = np.linspace(0.0, step_duration, x_ref.shape[0])
x_reconstruct = CubicHermiteSpline(dt, x_ref[:, :5], x_ref[:, 5:], axis=0)
u_reconstruct = CubicSpline(dt, u_ref, axis=0)

# (2.b) initialize pinocchio controller
controller = "TVLQR"

if controller == "PD":
    Kp = np.array([
            [100, 0,   0,  0],
            [0, 100, 0,  0],
            [0, 0,  100,  0],
            [0, 0,   0, 100]])
    Kd = np.array([
            [0.01, 0,  0,  0],
            [0,  0.01, 0,  0],
            [0,  0, 0.01,  0],
            [0,  0,  0, 0.01]])
    C = PDController(Kp, Kd)

elif controller == "FBL":
    # kp = np.diag([70,50,150,250])
    # kd = np.diag([10, 10, 10, 50])
    kp = np.diag([150,150,150,250])
    kd = np.diag([10, 10, 10, 50])
    C = FeedbackLinearization(biped, kp, kd)

elif controller == "TVLQR":
    Q = np.diag([900, 900, 900, 900, 900,
                 200, 200, 200, 200, 200])
    R = np.eye(x_ref.shape[1]//2 - 1) * 0.01
    C = TVLQR(biped, x_ref, u_ref, Q, R, step_duration, len(x_ref))

# (2.c) initialize motors
LK_MOTOR_ID = 0x1
RK_MOTOR_ID = 0x2
LH_MOTOR_ID = 0x3
RH_MOTOR_ID = 0x4
motors = CyberGear({LH_MOTOR_ID: "Cybergear", LK_MOTOR_ID: "Cybergear",
                RH_MOTOR_ID: "Cybergear", RK_MOTOR_ID: "Cybergear"})

motors.enable_for_motion()
print("CyberGear motors enabled.")

################## 3. Helper functions #####################
def _get_states():
    # gets motor position and velocity
    lk_data = motors.get_data(LK_MOTOR_ID)
    rk_data = motors.get_data(RK_MOTOR_ID)
    lh_data = motors.get_data(LH_MOTOR_ID)
    rh_data = motors.get_data(RH_MOTOR_ID)

    lankle = lk_data.current_position +  lh_data.current_position + rh_data.current_position + rk_data.current_position

    # Positions: [LK, LH, RH, RK]
    p_measured = np.array([-lankle, lk_data.current_position, lh_data.current_position,
                            rh_data.current_position, rk_data.current_position])

    # Velocities: [LK, LH, RH, RK]
    v_measured = np.array([0.0, lk_data.current_velocity, lh_data.current_velocity,
                            rh_data.current_velocity, rk_data.current_velocity])

    p_measured = -1*p_measured
    v_measured = -1*v_measured

    # print(f"p_measured: {p_measured}")
    # print(f"v_measured: {v_measured}")

    return p_measured, v_measured

def interpolation(t):
    p_ref = x_reconstruct(t)
    v_ref = x_reconstruct(t, 1)
    a_ref = x_reconstruct(t, 2)
    u_ref = u_reconstruct(t)
    return p_ref, v_ref, u_ref, a_ref

def _send_command(u):
    # motors.set_torque(LK_MOTOR_ID, u[1])
    # motors.set_torque(LH_MOTOR_ID, u[2])
    # motors.set_torque(RH_MOTOR_ID, u[3])
    # motors.set_torque(RK_MOTOR_ID, u[4])

    # time.sleep(0.002)
    # motors.set_torque(LK_MOTOR_ID, 0.0)

    print(f"commanded torque: {u}")

    return

class Logger:
    def __init__(self):
        self.time = []
        self.x_ref = []
        self.x_real = []
        self.u_ref = []
        self.u_real = []

    def log(self, t, x_ref, x_real, u_ref, u_real):
        self.time.append(t)
        self.x_ref.append(np.asarray(x_ref).copy())
        self.x_real.append(np.asarray(x_real).copy())
        self.u_ref.append(np.asarray(u_ref).copy())
        self.u_real.append(np.asarray(u_real).copy())

    def reset(self):
        self.__init__()

    def plot(self):
        time = np.asarray(self.time)
        x_ref = np.asarray(self.x_ref)
        x_real = np.asarray(self.x_real)
        u_ref = np.asarray(self.u_ref)
        u_real = np.asarray(self.u_real)

        state_names = ["Left ankle", "Left knee", "Left hip", "Right hip", "Right knee"]

        actuator_names = ["Left knee", "Left hip", "Right hip", "Right knee"]

        # for i, name in enumerate(state_names):
        #     plt.figure(figsize=(10, 4))
        #     plt.plot(time, x_ref[:, i], label="Reference")
        #     plt.plot(time, x_real[:, i], "--", label="Measured")
        #     plt.xlabel("Time [s]")
        #     plt.ylabel("Position [rad]")
        #     plt.title(name)
        #     plt.grid()
        #     plt.legend()

        for i, name in enumerate(actuator_names):
            plt.figure(figsize=(10, 4))
            plt.plot(time, u_ref[:, i], label="Feedforward reference")
            plt.plot(time, u_real[:, i], "--", label="Applied torque")
            plt.xlabel("Time [s]")
            plt.ylabel("Torque [Nm]")
            plt.title(f"{name} torque")
            plt.grid()
            plt.legend()

        plt.show()


################## 4. Main control loop #####################
# (4.a) set initial states
input("Press Enter to configure robot to its initial pose.")
row_init = x_ref[0]
lknee_angle_init = row_init[1]
lhip_angle_init = row_init[2]
rhip_angle_init = row_init[3]
rknee_angle_init = row_init[4]

# motors.set_position(LH_MOTOR_ID, lhip_angle_init,kp=3,kd=0.2)
# motors.set_position(LK_MOTOR_ID, lknee_angle_init,kp=3,kd=0.2)
# motors.set_position(RH_MOTOR_ID, rhip_angle_init,kp=3,kd=0.2)
# motors.set_position(RK_MOTOR_ID, rknee_angle_init,kp=3,kd=0.2)

# (4.b) control loop
CONTROL_DT = 0.002

def main():
    logger = Logger()

    start_time = time.perf_counter()
    next_step = start_time

    while True:
        now = time.perf_counter()
        t = now - start_time

        if t >= dt[-1]:
            break

        p_measured, v_measured = _get_states()
        p_ref, v_ref, uff, a_ref = interpolation(t)
        
        if controller == "PD":
            u = C.compute(p_ref, v_ref, p_measured, v_measured, uff, a_ref)

        elif controller == "TVLQR" or controller == "FBL":
            u = C.compute(p_ref, v_ref, p_measured, v_measured, uff, a_ref, t)

        u = np.clip(u, -4.0, 4.0)
        _send_command(u)
        
        next_step += CONTROL_DT
        
        sleep_time = next_step - time.perf_counter()
        if sleep_time > 0:
            time.sleep(sleep_time)


if __name__ == "__main__":
    main()
