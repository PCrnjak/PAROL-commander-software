# PAROL-commander-software
[![License: MIT](https://img.shields.io/badge/license-GPLv3-blue)](https://opensource.org/license/gpl-3-0/)

<img src="Images/screen_2.png" alt="drawing" width="700"/>

https://source-robotics.com

Robot arm building instructions, STL files, BOM, PAROL6 control board code and schematics can be found here: [Link](https://github.com/PCrnjak/PAROL6-Desktop-robot-arm)

PAROL6 comamnder software is tool for controling and programming PAROL6 robotic arm!
Some of the features are: <br />
* Loop rates of up to 100Hz allowing for real time control
* Joint jogging
* Cartesian level jogging in world reference frame and tool reference frame
* Full telemetry data
* Error handling like Inverse kinematics errors, Joint position limits, speed limits, Estop...
* Response log 
* Built in simulator
* Programming interface with simple to use commands
* Real time control of robot outputs and reading of inputs



# What is PAROL6 robotic arm?

<img src="Images/img3.png" alt="drawing" width="5000"/>

PAROL6 is a high-performance 3D-printed desktop robotic arm. The design approach of PAROL6 was to be similar to industrial robots in terms of mechanical design, control software, and usability. Control software, GUI, and robots STL files are open-source. You can build your own PAROL6 robot by following the instructions on this page.

Github link for PAROL6 robotic arm is here!

# How to install

  sudo apt install python3
  ---> CURRENT VESRION IS INSTALLED: python3 is already the newest version (3.10.6-1~22.04).
  sudo apt install python3-pip
  sudo apt-get install git
  pip3 install git+https://github.com/PCrnjak/s_visual_kinematics.git@main#egg=s_visual_kinematics
  pip3 install customtkinter
  pip3 install customtkinter --upgrade
  pip3 install oclock
  pip3 install serial
  git clone https://github.com/petercorke/robotics-toolbox-python.git
  cd robotics-toolbox-python
  pip3 install -e .
  pip3 install swift-sim
  sudo apt-get install python3-tk
  pip3 install pgraph-python
  pip3 install progress
  sudo apt-get install python3-pil python3-pil.imagetk

## Clone
git clone https://github.com/PCrnjak/PAROL-commander-software.git

## To run go to install  directory and
  python3 Serial_sender_good_latest.py


## If using visual studio code
  install python extension

## If getting serial errors try:
  sudo chmod 666 /dev/ttyAMA0

## To find serial device try this:
https://askubuntu.com/questions/398941/find-which-tty-device-connected-over-usb

# Install via pip
```
pip3 install git+https://github.com/PCrnjak/s_visual_kinematics.git@main#egg=s_visual_kinematics
```

# Install via Github

```
git clone https://github.com/PCrnjak/s_visual_kinematics.git
cd s_visual_kinematics
pip3 install -e .
```


# How to run


# Dependency


# Documentation:

How to use PAROL commander software can be found in [DOCS](https://pcrnjak.github.io/PAROL-docs/)


# More about PAROL6
Join [Discord](https://discord.com/invite/prjUvjmGpZ ) community!
- [Youtube](https://www.youtube.com/channel/UCp3sDRwVkbm7b2M-2qwf5aQ)
- [Hackaday](https://hackaday.io/project/167247-faze4-robotic-arm)
- [Instagram](https://www.instagram.com/5arcrnjak/)
- [DOCS](https://source-robotics.github.io/PAROL-docs/)


# Support the project

All the code and STL files needed to build the robot are Open source and free to all and I would like to keep it that way. Any help 
in terms of donations, advice, or contribution is really appreciated. Thank you!


[<img src="Images/Donate.png" width="200">](http://google.com.au/)


# Project is under GPLv3 Licence
