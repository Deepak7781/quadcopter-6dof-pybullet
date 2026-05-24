import pybullet as pb
import pybullet_data
import time
import numpy as np
from math import sin, cos, tan

pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())

pb.setGravity(0, 0, -9.81)
pb.loadURDF("plane.urdf")

# Drone dimensions
arm_length = 0.8
body_size = 0.15
rotor_r = 0.12
rotor_t = 0.01

# Visual shapes
central_body_visual = pb.createVisualShape(
    shapeType=pb.GEOM_BOX,
    halfExtents=[body_size, body_size, 0.03],
    rgbaColor=[0.5, 0.5, 0, 1]
)

central_body_collision = pb.createCollisionShape(
    shapeType=pb.GEOM_BOX,
    halfExtents=[body_size, body_size, 0.03]
)

arm_visual = pb.createVisualShape(
    shapeType=pb.GEOM_BOX,
    halfExtents=[arm_length/2, 0.03, 0.02],
    rgbaColor=[0.2, 0.8, 0, 1]
)

rotor_visual = pb.createVisualShape(
    shapeType=pb.GEOM_CYLINDER,
    radius=rotor_r,
    length=rotor_t,
    rgbaColor=[0.3, 0.4, 0.7, 1]
)

# Rotor positions
L = arm_length / 2
rotor_positions = [
    [ L/np.sqrt(2),  L/np.sqrt(2), 0.04],
    [ L/np.sqrt(2), -L/np.sqrt(2), 0.04],
    [-L/np.sqrt(2),  L/np.sqrt(2), 0.04],
    [-L/np.sqrt(2), -L/np.sqrt(2), 0.04]
]

arm_orn_1 = pb.getQuaternionFromEuler([0, 0, np.pi/4])
arm_orn_2 = pb.getQuaternionFromEuler([0, 0, -np.pi/4])

drone = pb.createMultiBody(
    baseMass=5.0,
    baseCollisionShapeIndex=central_body_collision,
    baseVisualShapeIndex=central_body_visual,
    basePosition=[0, 0, 2],

    linkMasses=[0, 0, 0, 0, 0, 0],
    linkCollisionShapeIndices=[-1]*6,
    linkVisualShapeIndices=[
        arm_visual, arm_visual,
        rotor_visual, rotor_visual, rotor_visual, rotor_visual
    ],

    linkPositions=[
        [0, 0, 0], [0, 0, 0],
        rotor_positions[0],
        rotor_positions[1],
        rotor_positions[2],
        rotor_positions[3]
    ],

    linkOrientations=[
        arm_orn_1,
        arm_orn_2,
        [0, 0, 0, 1],
        [0, 0, 0, 1],
        [0, 0, 0, 1],
        [0, 0, 0, 1]
    ],

    linkInertialFramePositions=[[0, 0, 0]]*6,
    linkInertialFrameOrientations=[[0, 0, 0, 1]]*6,
    linkParentIndices=[0]*6,
    linkJointTypes=[pb.JOINT_FIXED]*6,
    linkJointAxis=[[0, 0, 1]]*6
)


# Drone Dynamics

m = 5 # mass of the drone
Ixx = 0.1
Iyy = 0.1
Izz = 0.2
g = 9.81 # gravitational acceleration

# Drone Actuator limits
THRUST_MAX = 2.0 * m * g
THRUST_MIN = 0.0
MOMENT_MAX = 5.0
VRATE_MAX = 5.0
ANGLE_MAX = np.radians(30)

# Initial States
u = 0 
v = 0 
w = 0 

p = 0 
q = 0 
r = 0 

phi = 0
theta = 0
psi = 0

x = 0
y = 0
z = 0

# Desired States
z_des = 10
phi_des = 0
theta_des = 0
psi_des = 0
x_des = 10
y_des = 0

# Values for plotting
x_hist = [x]
y_hist = [y]
z_hist = [z]

phi_hist = [phi]
theta_hist = [theta]
psi_hist = [psi]
phi_des_hist = [phi_des]
theta_des_hist = [theta_des]
# For PID control
zError_int = 0
zError_prev = 0

phiError_int = 0
phiError_prev = 0

thetaError_int = 0
thetaError_prev = 0

psiError_int = 0
psiError_prev = 0

# PID gains
# Altitude Control - Cascaded PID

# Outer Loop: altitude error -> desired vertical velocity
Kp_z = 0.5

# Inner Loop: vertical velocity error -> thrust correction
Kp_zdot = 8
Ki_zdot = 2
Kd_zdot = 4


# Attitude Control - Single Loop PID
Kp_phi = 9
Ki_phi = 0.05
Kd_phi = 4

Kp_theta = 4
Ki_theta = 0.05
Kd_theta = 4

Kp_psi = 6
Ki_psi = 0.02
Kd_psi = 2

# Position Controller Gains
Kp_x = 0.15
Kd_x = 0.35

Kp_y = 0.15
Kd_y = 0.35

xdot = 0 
ydot = 0
zdot = 0

if z > 0:  
    u1_hist    = [m * g]   # hover thrust at t=0
else:
    u1_hist    = [0.0]     # no thrust needed if already on the ground

T = 50
dt = 0.001
Tsim = np.arange(0, T, dt)

z_e = z_des - z
phi_e = phi_des - phi
theta_e = theta_des - theta
psi_e = psi_des - psi


phiError_deriv = 0
thetaError_deriv = 0
psiError_deriv = 0
zdotError_int = 0
zdot_prev = zdot


zdot_hist = [zdot]
zdot_des_hist = [0]

for i in range(1, len(Tsim)):
    x_e = x_des - x
    y_e = y_des - y

    ax_des = Kp_x * x_e + Kd_x * (0 - xdot)
    ay_des = Kp_y * y_e + Kd_y * (0 - ydot)

    # Convert desired acceleration to desired tilt angles

    theta_des = (ax_des * cos(psi) + ay_des * sin(psi)) / g

    phi_des = (ax_des * sin(psi) - ay_des * cos(psi)) / g

    # Limit commanded tilt angle
    phi_des = np.clip(phi_des, -ANGLE_MAX, ANGLE_MAX)
    theta_des = np.clip(theta_des, -ANGLE_MAX, ANGLE_MAX)
    phiError_int = phiError_int + phi_e * dt
    thetaError_int = thetaError_int + theta_e * dt
    psiError_int = psiError_int + psi_e * dt

    phiError_prev = phi_e
    thetaError_prev = theta_e
    psiError_prev = psi_e

    zdot_des = Kp_z * z_e
    zdot_des = np.clip(zdot_des, -VRATE_MAX, VRATE_MAX)

    zdot_e = zdot_des - zdot
    zdotError_int = zdotError_int + zdot_e * dt
    zdotError_int = np.clip(zdotError_int, -5, 5)

    zdot_deriv = -(zdot - zdot_prev) / dt
    zdot_prev = zdot

    u1 = (m*g + Kp_zdot*zdot_e + Ki_zdot*zdotError_int + Kd_zdot*zdot_deriv) / (cos(phi)*cos(theta))
    u2 = Kp_phi*phi_e + Ki_phi*phiError_int + Kd_phi*phiError_deriv
    u3 = Kp_theta*theta_e + Ki_theta*thetaError_int + Kd_theta*thetaError_deriv
    u4 = Kp_psi*psi_e + Ki_psi*psiError_int + Kd_psi*psiError_deriv

    u1 = np.clip(u1, THRUST_MIN, THRUST_MAX)
    u2 = np.clip(u2, -MOMENT_MAX, MOMENT_MAX)
    u3 = np.clip(u3, -MOMENT_MAX, MOMENT_MAX)
    u4 = np.clip(u4, -MOMENT_MAX, MOMENT_MAX)

    u1_hist.append(u1)

    # Translational Equations of Motion in Earth Frame
    xddot = (u1/m) * (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi))
    yddot = (u1/m) * (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi))
    zddot = (u1/m) * (cos(phi)*cos(theta)) - g

    # Rotational Equations of Motion in Body Frame
    pdot = (u2 + q*r*(Iyy - Izz))/Ixx
    qdot = (u3 + p*r*(Izz - Ixx))/Iyy
    rdot = (u4 + p*q*(Ixx - Iyy))/Izz

    # Euler Angles
    phidot = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta)
    thetadot = q*cos(phi) - r*sin(phi)
    psidot = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta)

    # Euler integration
    xdot = xdot + xddot * dt
    ydot = ydot + yddot * dt
    zdot = zdot + zddot * dt

    x = x + xdot * dt
    y = y + ydot * dt
    z = z + zdot * dt

    p = p + pdot * dt
    q = q + qdot * dt
    r = r + rdot * dt

    phi = phi + phidot * dt
    theta = theta + thetadot * dt
    psi = psi + psidot * dt

    z_e = z_des - z
    phi_e = phi_des - phi
    theta_e = theta_des - theta
    psi_e = psi_des - psi

    zError_deriv = (z_e - zError_prev) / dt
    phiError_deriv = (phi_e - phiError_prev) / dt
    thetaError_deriv = (theta_e - thetaError_prev) / dt
    psiError_deriv = (psi_e - psiError_prev) / dt

    if i % 10 == 0:
        orn = pb.getQuaternionFromEuler([phi, theta, psi])
        pb.resetBasePositionAndOrientation(drone, [x, y, z], orn)

        pb.resetDebugVisualizerCamera(
            cameraDistance=8,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[x, y, z]
        )

        pb.stepSimulation()
        time.sleep(dt * 10)

pb.disconnect()