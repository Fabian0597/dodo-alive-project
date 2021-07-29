# Dodo Alive Project

## Introduction
In this project a SLIP model was used to control a robotic articulated leg. The articulated leg is simulated as a LUA model. The RBDL library, which offers efficient rigid body dynamics algorithms is used. The articulated leg is visualized with MeshUp, a tool that was developed with RBDL. Feel free to read our detailed report in the repository.

<table style="margin-left: auto; margin-right: auto; table-layout: fixed; width: 100%">
  <tr>
    <td style="width: 48%;"> <img src="own-project/document/ressources/jumping_RK45.gif " width="700"/></td>
  </tr>
  <tr>
    <td style="width: 48%;" valign="top"> <b>GIF 1:</b> Articulated leg with RKF45 integrator.
    </td>
  </tr>
</table>

## SLIP model
The spring-loaded-inverted-pendulum (SLIP) models the leg with a mass-less spring of resting length of l0 and a stiffness k. A point mass is attached at the top of the spring. In the flight phase, the SLIP model follows the law of gravity, moving on a parabolic trajectory. When touching the ground, the motion of the center of gravity (CoG) is redirected by the spring force in the leg, which acts between the foot contact point and the CoG. The SLIP model is a very simple and abstract description of the spring-leg behavior in human running. We can make use of this model by projecting the control applied to the SLIP model onto the dynamics of an actual segmented robotic leg. The low level SLIP model generates a closed-loop dynamics, which is tracked in the high level robot model. Feedback control can be used to adapt to perturbation. Since the SLIP model is a conservative system the forward speed and the apex height, which characterize the trajectory are coupled. The stability problem therefore is reduced to one dimension and the leg model can be solely controlled with the landing angle.

## Energy compensation
In comparison to a SLIP model with a mass-less spring, real robots are influenced from the impact when the leg strikes the ground. This is the reason why the velocity of the CoG of the robot changes through the transition from flight to stance. This corresponds to a loss of kinetic energy. In the modified SLIP model with impact compensation, the spring leg is pre-compressed at touch down. The energy stored in the pre-compressed spring equals the kinetic energy loss and therefore compensates for that loss. In the following implementation, the modified SLIP model is used. We used the paper "Hutter et al. 2010 - SLIP running with an Articulated Robotic Leg" as orientation.

## Control law 
We separate the control laws in the flight phase from that in the stance phase. For the flight, phase a cascade control using PI and PID controller is used. For the stance phase, a SLIP model control law is used.

### Flight control
During flight, the leg length and the landing angle are held constant. For this, a cascade control using two PI and one PID controller are used.

<table style="margin-left: auto; margin-right: auto; table-layout: fixed; width: 100%">
  <tr>
    <td style="width: 48%;"> <img src="own-project/document/ressources/cascade_control.png " width="600"/></td>
  </tr>
  <tr>
    <td style="width: 48%;" valign="top"> <b>IMG 1:</b> Cascade contol.
    </td>
  </tr>
</table>

## Structure
### Hybrid automaton
<table style="margin-left: auto; margin-right: auto; table-layout: fixed; width: 100%">
  <tr>
    <td style="width: 48%;"> <img src="own-project/document/ressources/hybrid_automaton.png " width="600"/></td>
  </tr>
  <tr>
    <td style="width: 48%;" valign="top"> <b>IMG 2:</b> Hybrid automoton.
    </td>
  </tr>
</table>

### Implementation
An automaton was used to capture the robot state. It contains a continous state (generalized coordinates), which consists of the  coordinates of the floating base in the 3D space, the hip angle and knee angle. It also contains the discrete state, which describes whether the robot is in the flight or stance phase. The transitions between these states are described through events. For each of these two discrete states an own controller class is implemented, which are described above.

<table style="margin-left: auto; margin-right: auto; table-layout: fixed; width: 100%">
  <tr>
    <td style="width: 24%;"> <img src="own-project/document/ressources/class_structure.png " width="600"/></td>
  </tr>
  <tr>
    <td style="width: 48%;" valign="top"> <b>IMG 3:</b> Class diagram.
    </td>
  </tr>
</table>


### Stance control
When touching the ground, the SLIP dynamics is projected onto the CoG motion of the robotic leg. From there, the required joint actuator torques in order to generate the necessary operational space forces are calculated. We used the paper "Hutter et al. 2010 - SLIP running with an Articulated Robotic Leg" as orientation.


## Solver

To calculate the robot's state we iteratively do a control, forward dynamics and integration step. In the control step, we calculate the required robot actuation from the current robot state. For this we use our stance and flight controller. In the forward dynamics, the robot state derivative is calculated from the current robot state and the robot actuation. Integrating this robot state derivative returns the robot state for the next time step. In each iteration, a state transition from stance to flight or flight to stance phase is checked. The jump function uses the solved robot state from this solver iteration to calculate the discrete state transition. To integrate the generalized accelerations, two different solvers were implemented. In a first attempt, the ivp_solver from scipy is used. This solver integrates the robot state derivative with a variable stepsize. Root-finding helps to find the exact state transition. In a second attempt, the RKF45 algorithm is implemented, which finds a numerical solution of ordinary differential equations. It uses a fixed step size. With a fixed step size the differential part of the PID controller in the flight phase works more stable. GIF 1 shows the RKF45 solver which works a lot better than the ivp_solver shown in GIF 2.

<table style="margin-left: auto; margin-right: auto; table-layout: fixed; width: 100%">
  <tr>
    <td style="width: 48%;"> <img src="own-project/document/ressources/jumping_ivp.gif " width="800"/></td>
  </tr>
  <tr>
    <td style="width: 48%;" valign="top"> <b>GIF 2:</b> Articulated leg with ivp_solver.
    </td>
  </tr>
</table>


## Setup the Project

```
apt update && apt install -y \
  lua5.1 \
  liblua5.1-0-dev \
  git \
  git-core \
  cmake \
  cmake-curses-gui \
  build-essential \
  libeigen3-dev
apt install -y \
  ffmpeg \
  libavutil-dev \
  libavcodec-dev \
  libavutil-dev \
  libavformat-dev \
  libswscale-dev \
  libvtk6-qt-dev \
  libboost-all-dev
apt install -y \
  python-dev \
  build-essential \
  python3-dev \
  python3-pip

# install programs
apt -y install qtcreator
apt -y install qt5-default python3-pip 
pip3 install Cython
pip3 install scipy
```

For computing the kinematics and dynamics of robot - install the Rigid Body Dynamics Library (RBDL)
(Download RBDL from the moodle course)
```
cd /home/appuser/rbdl-tum
mkdir build
cd build
ccmake ..
```

Hit "c" repeatedly to configure and ensure that the following options are set:
```
CMAKE_BUILD_TYPE		 Release
RBDL_BUILD_ADDON_LUAMODEL           ON
RBDL_BUILD_ADDON_GEOMETRY           ON
RBDL_BUILD_ADDON_URDFREADER         ON                                                                                                
RBDL_BUILD_PYTHON_WRAPPER           ON
```
Then hit "g" to generate and exit.

```
make
sudo make install
```

For visualization, we will use MeshUp - a tool that was developed with RBDL
```
cd /home/appuser/git
git clone https://github.com/ORB-HD/MeshUp
cd MeshUp
mkdir build
cd build
ccmake ..
```

Again hit "c", "g"

```
make
sudo make install
```

Add rbdl to PYTHONPATH

add the following line to the file `~/.bashrc`:
```
export PYTHONPATH=$PYTHONPATH:~/rbdl-tum/build/python
```

## Run the Program

Run the motion simulation:
```
/usr/bin/python3.8 /home/appuser/git/own-project/run_articulated_leg_walker.py
```

Show the result:
```
meshup model/articulatedLeg.lua animation.csv
```

## Meetings

Weekly Meetings on Monday 10:00 am (14.06 - 26.06):

```
Zoom-Meeting beitreten
https://tum-conf.zoom.us/j/63721696895

Meeting-ID: 637 2169 6895
Kenncode: Dodo2021
```



