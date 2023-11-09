# How to install Linux

    sudo apt install python3
    ---> CURRENT VERSION INSTALLED: python3 is already the newest version (3.10.6-1~22.04).
    sudo apt install python3-pip
    sudo apt-get install git
    pip3 install git+https://github.com/PCrnjak/s_visual_kinematics.git@main#egg=s_visual_kinematics
    pip3 install customtkinter
    pip3 install customtkinter --upgrade
    pip3 install oclock
    pip3 install pyserial
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

## To run go to the install directory and
    python3 Serial_sender_good_latest.py


## If using Visual Studio code
    install python extension

## If getting serial errors try:
    sudo chmod 666 /dev/ttyAMA0

## To find the serial device try this:
https://askubuntu.com/questions/398941/find-which-tty-device-connected-over-usb