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
    pip3 install roboticstoolbox-python==1.0.3
    pip3 install swift-sim==1.0.1
    sudo apt-get install python3-tk
    pip3 install pgraph-python
    pip3 install progress 
    sudo apt-get install python3-pil python3-pil.imagetk
    pip3 install numpy==1.23.4
    pip3 install scipy==1.11.4

## Clone
git clone https://github.com/PCrnjak/PAROL-commander-software.git

## To run go to the install directory and
    python3 Serial_sender_good_latest.py


## If using Visual Studio code
    install python extension

## If getting serial errors try (0 might be some other number depending on what com port your robot was assigned to):
    sudo chmod 666 /dev/ttyACM0 

## To find the serial device try this:
https://askubuntu.com/questions/398941/find-which-tty-device-connected-over-usb

## If connecting over GUI enter (x is your port number):
    ttyACMx 
Press connect multiple times

## Troubleshooting
Check In https://github.com/PCrnjak/PAROL-commander-software/tree/main/Working%20dependency what python module versions were used on working systems
