# Dodo Alive Project

## Introduction
In this Project a SLIP model was projected onto the dynamics of an robotic articulated leg. The articulated leg is simulated as a LUA model. RBDL, a library for efficient rigid body dynamics algorithms is used for calculating Jacobians, transformations between coordiante frames, forward dynamics, computing the contact gain for the ground impact, constraining motion and position in the stance phase. For visualization, we will use MeshUp; a tool that was developed with RBDL.

<table style="margin-left: auto; margin-right: auto; table-layout: fixed; width: 100%">
  <tr>
    <td style="width: 48%;"> <img src="own-project/document/ressources/jumping_RK45.gif " width="800"/></td>
  </tr>
  <tr>
    <td style="width: 48%;" valign="top"> <b>GIF 1:</b> jumping robot.
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



