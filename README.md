# px4-offboard
This `repository` contains a python examples for offboard control on ROS2 with [PX4](https://px4.io/)

The `px4_offboard` package contains the following nodes
- `offboard_control.py`: Example of offboard position control using position setpoints
- `visualizer.py`: Used for visualizing vehicle states in Rviz

The source code is released under a BSD 3-Clause license.

- **Author**: Jaeyoung Lim
- **Affiliation**: Autonomous Systems Lab, ETH Zurich

## 修改成MAVROS通訊機制
### 使用方式
1. Requirements：ROS2 Humble [Ubuntu (deb packages) — ROS 2 Documentation: Humble documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
2. 建置ROS APP  
    設定ROS環境變數
    ```
    source /opt/ros/humble/setup.bash
    ```
    開始build
    ```
    cd px4_offboard
    colcon build --packages-select px4_offboard --symlink-install
    ```
3. 運行ROS node  
    設定ROS APP環境變數
    ```
    source install/setup.bash
    ```
    運行ROS node
    ```
    ros2 launch px4_offboard offboard_position_control_mavros_auto.launch.py
    ```

### Laptop + MTK edge computer + X-Plane demo使用方式
1. Requirements：MAVROS
2. WSL Networking設定: 開啟WSL Settings
    1. 設定網路模式(networkingMode)為Mirrored:  
    網路功能 > 網路模式 > 選擇Mirrored
    2. 設定主機位址回送(hostAddressLoopback)功能:  
    網路功能 > 主機位址回送 > 開啟
    3. 允許Hyper-V防火牆:  
        Powershell執行
        ```
        Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
        ```
3. Windows防火牆設定
    1. 開啟「具有進階安全性的Windows Defender防火牆」
    2. 新增規則
    3. 規則類型：選擇「連接埠」
    4. 通訊協定及連接埠：選擇UDP；特定本機連接埠14500-14599
    5. 動作：允許連線
    6. 設定檔：所有
    7. 名稱：自訂
4. 在運行px4xplane的電腦上(在同一個網路)
    1. 開啟X-Plane
    2. 執行px4xplane隨附的PX4(記得指定正確的機型)
        ```
        cd PX4-Autopilot-Me
        make px4_sitl xplane_alia250
        ```
5. 在MTK邊緣裝置上(在同一個網路)
    1. 在一個terminal開啟MAVROS node
        ```
        source configure_ros_dds.sh
        ros2 launch mavros px4.launch fcu_url:=udp://:14540@<PX4_HOST_IP>:14580
        
        # PX4_HOST_IP：運行px4xplane的IP
        ```
    2. 開新的terminal運行px4-offboard node
        ```
        source configure_ros_dds.sh
        ros2 launch px4_offboard offboard_position_control_mavros_auto.launch.py
        ```
    3. 執行接收GCS指令的Python script (待更新)
        ```
        source configure_ros_dds.sh
        python3 plane_control_node_v4_custom.py
        ```
6. 在Laptop上(在同一個網路)
    1. 執行地面站的Python script (待更新)
        ```
        source configure_ros_dds.sh
        python3 gcs_tui_v2.py
        ```
7. 設定無人機開始繞圈飛行：
    無人機arm並設定為offboard模式之後，將會開始繞圈飛行。需要在Laptop上執行`gcs_tui_v2.py`的terminal，按"a" + Enter解鎖，再按"m" + Enter進入offboard模式。

## Setup
Add the repository to the ros2 workspace
```
git clone https://github.com/Jaeyoung-Lim/px4-offboard.git
```

If you are running this on a companion computer to PX4, you will need to build the package on the companion computer directly. 

## Running

### Software in the Loop
You will make use of 3 different terminals to run the offboard demo.

On the first terminal, run a SITL instance from the PX4 Autopilot firmware.
```
make px4_sitl gz_x500
```

On the second terminal terminal, run the micro-ros-agent which will perform the mapping between Micro XRCE-DDS and RTPS. So that ROS2 Nodes are able to communicate with the PX4 micrortps_client.
```
MicroXRCEAgent udp4 -p 8888
```

In order to run the offboard position control example, open a third terminal and run the the node.
This runs two ros nodes, which publishes offboard position control setpoints and the visualizer.
```
ros2 launch px4_offboard offboard_position_control.launch.py
```
![offboard](https://user-images.githubusercontent.com/5248102/194742116-64b93fcb-ec99-478d-9f4f-f32f7f06e9fd.gif)

In order to just run the visualizer,
```
ros2 launch px4_offboard visualize.launch.py
```
### Hardware

This section is intended for running the offboard control node on a companion computer, such as a Raspberry Pi or Nvidia Jetson/Xavier. You will either need an SSH connection to run this node, or have a shell script to run the nodes on start up. 

If you are running this through a UART connection into the USB port, start the micro-ros agent with the following command

```
micro-ros-agent serial --dev /dev/ttyUSB0 -b 921600 -v
```
If you are using a UART connection which goes into the pinouts on the board, start the micro-ros agent with the following comand
```
micro-ros-agent serial --dev /dev/ttyTHS1 -b 921600 -V
```

To run the offboard position control example, run the node on the companion computer
```
ros2 launch px4_offboard offboard_hardware_position_control.launch.py
```

### Visualization parameters

The following section describes ROS2 parameters that alter behavior of visualization tool.


#### Automatic path clearing between consequent simulation runs

Running visualization node separately from the simulation can be desired to iteratively test how variety of PX4 (or custom offboard programs) parameters adjustments influence the system behavior. 

In such case RVIZ2 is left opened and in separate shell simulation is repeadetly restarted. This process causes paths from consequent runs to overlap with incorrect path continuity where end of previous path is connected to the beginning of new path from new run. To prevent that and clear old path automatically on start of new simulation, set `path_clearing_timeout` to positive float value which corresponds to timeout seconds after which, upon starting new simulation, old path is removed.

As an example, setting the parameter to `1.0` means that one second of delay between position updates through ROS2 topic will schedule path clearing right when next position update comes in (effectively upon next simulation run).


To enable automatic path clearing without closing visualization node set the param to positive floating point value:
```
ros2 param set /px4_offboard/visualizer path_clearing_timeout 1.0
```

To disable this feature set timeout to any negative floating point value (the feature is disabled by default):
```
ros2 param set /px4_offboard/visualizer path_clearing_timeout -1.0
```
