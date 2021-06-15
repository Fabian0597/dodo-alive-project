# Rbdl-Python

We use a modified version of the python wrapper provided by https://github.com/ORB-HD/rbdl-orb for simulating and controlling robots

## Get started

## Manual 
execute the following in the rbdl-orb folder
```sh
sudo pip3 install numpy
mkdir build
cd build
cmake ..
ccmake .. #see bellow
sudo make install
```
for ccmake:
- press c to config
- "ON" at RBDL_BUILD_ADDON_GEOMETRY
- "ON" at RBDL_BUILD_ADDON_LUAMODEL
- "ON" at RBDL_BUILD_PYTHON_WRAPPER
- press c again 2 times
- g to generate and exit

### rebuild
execute the following in the rbdl-orb folder
```sh
sudo chmod +x dobuild.sh # only required the first time
sudo sh dobuild.sh
```

## Modify Python wrapper

install packages needed
```sh
sudo apt-get install python-dev build-essential
sudo pip3 install Cython
sudo pip3 install scipy
```