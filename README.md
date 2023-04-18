## bno080_micro_ros
ROS2 driver for BNO080 interfaced with a microROS compatible micro-controller. The IMU data is acquired using this Sparkfun [Library](https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library)

## 1. Installation
### 1.1 Install the ROS2 driver and its dependencies:

    cd <your_ws>
    git clone https://github.com/grassjelly/bno080_micro_ros src/bno080_micro_ros
    rosdep install --from-paths src -i -r -y
    colcon build

## 2. Installing the firmware

### 2.1 Install PlatformIO (only have to do once)

    python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio/master/scripts/get-platformio.py)"
    echo "PATH=\"\$PATH:\$HOME/.platformio/penv/bin\"" >> $HOME/.bashrc
    source $HOME/.bashrc

### 2.2 Install Teensy's UDEV rules:

    cd /tmp
    wget https://www.pjrc.com/teensy/00-teensy.rules
    sudo cp 00-teensy.rules /etc/udev/rules.d/
    udevadm control --reload-rules && udevadm trigger

### 2.3 Install the firmware to the Teensy Board

    cd bno080_micro_ros/firmware
    pio run --target upload
    
* You might need to press the reset button on the Teensy if it's the first time you're uploading the firmware or when you get this error `Found device but unable to open
`

## 3. Running the driver

### 3.1 Run the launch file to run the [microROS agent](https://github.com/micro-ROS/micro-ROS-Agent):

    ros2 launch bno080_micro_ros imu.launch

Optional arguments

**rviz**  - If you want to run rviz and visualize the filter's estimated pose. For example:

    ros2 Launch bno080_micro_ros imu.launch rviz:=true

