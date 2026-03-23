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
    ```bash
    source /opt/ros/humble/setup.bash
    ```
    開始build
    ```bash
    cd px4_offboard
    colcon build --packages-select px4_offboard --symlink-install
    ```
3. 運行ROS node  
    設定ROS APP環境變數
    ```bash
    source install/setup.bash
    ```
    運行ROS node
    ```bash
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
        ```bash
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
        ```bash
        cd PX4-Autopilot-Me
        make px4_sitl xplane_alia250
        ```
    3. px4xplane puglin連線至SITL
5. 在MTK邊緣裝置上(在同一個網路)
    1. 在一個terminal開啟MAVROS node
        ```bash
        source configure_ros_dds.sh
        ros2 launch mavros px4.launch fcu_url:=udp://:14540@<PX4_HOST_IP>:14580
        
        # PX4_HOST_IP：運行px4xplane的IP
        ```
        補充：所有運行ROS的terminal都需要跑過`configure_ros_dds.sh`，這是用來設定ROS通訊的變數
    2. 開新的terminal運行px4-offboard node
        ```bash
        source configure_ros_dds.sh
        ros2 launch px4_offboard offboard_position_control_mavros_auto.launch.py
        ```
    3. 執行接收GCS指令的Python script
        ```bash
        source configure_ros_dds.sh
        python3 plane_control_node_v4_custom.py
        ```
6. 在Laptop上(在同一個網路)
    1. 執行地面站的Python script
        ```bash
        source configure_ros_dds.sh
        python3 gcs_tui_v2.py
        ```
7. 設定無人機開始繞圈飛行：
    無人機arm並設定為offboard模式之後，將會開始繞圈飛行。  
    需要在Laptop上執行`gcs_tui_v2.py`的terminal，按"a" + Enter解鎖，再按"m" + Enter進入offboard模式。  
    補充可用之指令：
    ```python
    '=== PX4 Plane GCS TUI v2 ==='
    'a=ARM  z=DISARM  t=TAKEOFF'
    'r=ROLL+10  l=ROLL-10  o=ROLL OFF'
    'u=PITCH+10  d=PITCH-10  p=PITCH OFF'
    '1=THR 30%  2=THR 50%  3=THR 70%'
    '+=THR UP  -=THR DOWN  0=THR OFF'
    'm=OFFBOARD  q=QUIT'
    ```

## px4xplane設定新機型
1. 修改`config.ini`
    1. 撰寫檔案
        參考檔案：`config.ini`
        設定機型名稱：
        ```
        [FixedWing_DualMotor]
        ```
        設定引擎、舵面的DataRef：  
        - 格式：`channel{n} = {DataRef_name}, {data_type}, {index}, {range}`
        - `{n}`: 自己指定(之後跟PX4的參數對應就好)
        - `{DataRef_name}`: 引擎/舵面，PX4給DataRef值，X-Plane會做出相應的反應。  
            要找到有反應的DataRef，使用X-Plane安裝資料夾裡面的Plane Maker，找到哪些wing編號有功能，搭配DataRefTool去嘗試哪個DataRef有反應。  
            例如：Plane Maker > 上排選單 > Standard > Wings裡面顯示Wing 2有功能，再用DataRefTool搜尋wing2，找到有反應的DataRef name。
        - DataRef設定可以透過"|"符號串聯，被串的DataRef值會保持一致
        ```
        ; Enable auto prop brakes on X-Plane engines #1 and #2 (indices 0,1)
        autoPropBrakes = 0, 1

        ; Motors (PX4 channels -> X-Plane engine indices)
        ; Use ENGN_thro_use as floatArray per px4xplane docs
        channel0 = sim/flightmodel/engine/ENGN_thro_use, floatArray, [0], [-1 1]   ; Motor 1
        channel1 = sim/flightmodel/engine/ENGN_thro_use, floatArray, [1], [-1 1]   ; Motor 2

        ; Ailerons (Wing2)
        channel2 = sim/flightmodel/controls/wing2l_ail1def, float, 0, [-10 10]     ; Left aileron
        channel3 = sim/flightmodel/controls/wing2r_ail1def, float, 0, [-10 10]     ; Right aileron

        ; Elevator (Wing3) - drive both L/R from one PX4 channel (symmetric deflection)
        channel4 = sim/flightmodel/controls/wing3l_elv1def, float, 0, [10 -10] | sim/flightmodel/controls/wing3r_elv1def, float, 0, [10 -10]

        ; Drag Rudders / Yaw Brakes (Wing3) - left/right split
        ; X-Plane uses yawbrake defs (yawbdef). These provide yawing via drag.
        channel5 = sim/flightmodel/controls/wing3l_yawbdef, float, 0, [0 30]       ; Left rudder
        channel6 = sim/flightmodel/controls/wing3r_yawbdef, float, 0, [0 30]       ; Right rudder
        ```
        參考資料：[px4xplane/docs/custom-airframe-config.md at master · alireza787b/px4xplane](https://github.com/alireza787b/px4xplane/blob/master/docs/custom-airframe-config.md)
    2. 將檔案放至px4xplane的資料夾底下，例如：`"X-Plane 12/Resources/plugins/"`
2. 修改ROMFS配置檔案  
    1. 撰寫檔案  
        參考檔案：`22001_xplane_fly_wing`  
        通訊設定：
        - 格式：`param set-default PWM_MAIN_FUNC{m} {id}`
        - {m}：與`config.ini`的`channel{n}`對應，`m == n+1`、`m+1 == n+2`、依此類推
        - {id}：UAVCAN Motor Parameters - PCA9685_FUNC10。`101-112`為motors、`201-208`為servos，多個motor與servo只要id不重複就好，順序不影響。參考資料：[Parameter Reference | PX4 User Guide (v1.13)](https://docs.px4.io/v1.13/en/advanced_config/parameter_reference.html)
        ```bash
        # PWM Output Mapping
        # (Order matters for px4xplane channel mapping)
        # MAIN1..2: Ailerons, MAIN3: Elevator, MAIN4..5: Motors, MAIN6..7: Yawbrakes
        param set-default PWM_MAIN_FUNC1 101   # Motor 1 (X-Plane engine #1)
        param set-default PWM_MAIN_FUNC2 102   # Motor 2 (X-Plane engine #2)
        param set-default PWM_MAIN_FUNC3 201   # Left aileron
        param set-default PWM_MAIN_FUNC4 202   # Right aileron
        param set-default PWM_MAIN_FUNC5 203   # Elevator (single)
        param set-default PWM_MAIN_FUNC6 204   # Left rudder  (use spoiler func-id as generic)
        param set-default PWM_MAIN_FUNC7 205   # Right rudder (use spoiler func-id as generic)
        param set-default PWM_MAIN_REV   0
        ```
        其餘部分：沿用模板
    2. 將檔案放至路徑：`"PX4-Autopilot-Me/ROMFS/px4fmu_common/init.d-posix/airframes"`
3. 重新編譯PX4：
    ```bash
    cd PX4-Autopilot-Me
    rm -r build
    make clean

    make px4_sitl_default
    ```
    剛剛修改的ROMFS配置檔案應該會出現在`"PX4-Autopilot-Me/build/px4_sitl_default/rootfs/etc/init.d-posix/airframes"`裡面。
4. PX4執行(使用修改的ROMFS配置檔案)：
    ```bash
    PX4_SYS_AUTOSTART=<AIRFRAME_ID> make px4_sitl_default none
    ```
    `AIRFRAME_ID`：配置檔案的前綴數字，例如`22001_xplane_fly_wing`就是填`22001`

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