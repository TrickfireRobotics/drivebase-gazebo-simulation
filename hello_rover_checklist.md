# 🚀 Drivebase “Hello Rover” Practice — Checklist

## Before you start
1) **Create your own branch** off `main` named `practice/<your-name>`  
   ```bash
   git checkout -b practice/<your-name>
   git push -u origin practice/<your-name>
   ```

## Goal
- Spawn a simple diff-drive robot in **Gazebo Garden**.
- Bridge `/cmd_vel` between ROS 2 and Gazebo.
- Write a tiny ROS 2 node that publishes `/cmd_vel` to make the robot drive in a gentle circle.

---

## 0) Start the container
Follow the repo README for your OS. Then inside the container:

- **Choose one:**
  - **GUI (Terminal A):**
    ```bash
    gz sim -v 3 -r empty.sdf
    ```
  - **Headless (Terminal A):**
    ```bash
    gz sim -s -r empty.sdf
    ```

> You can also run Gazebo in the **background** in the same terminal by appending `&`, e.g. `gz sim -s -r empty.sdf &`, then continue with the next steps in that same terminal.

---

## 1) Add a minimal SDF model
Create the file below at:
```
ros2_ws/src/rover_description/models/tfr_bot.sdf
```

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <model name="tfr_bot">
    <link name="base_link">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial><mass>5.0</mass></inertial>
      <collision name="base_col">
        <geometry><box><size>0.4 0.3 0.1</size></box></geometry>
      </collision>
      <visual name="base_vis">
        <geometry><box><size>0.4 0.3 0.1</size></box></geometry>
      </visual>
    </link>

    <link name="wheel_left">
      <inertial><mass>0.5</mass></inertial>
      <collision name="wl_col">
        <geometry><cylinder><radius>0.05</radius><length>0.03</length></cylinder></geometry>
      </collision>
      <visual name="wl_vis">
        <geometry><cylinder><radius>0.05</radius><length>0.03</length></cylinder></geometry>
      </visual>
    </link>
    <joint name="left_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>wheel_left</child>
      <pose>0 0.15 0.05 0 1.5708 0</pose>
      <axis><xyz>0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
    </joint>

    <link name="wheel_right">
      <inertial><mass>0.5</mass></inertial>
      <collision name="wr_col">
        <geometry><cylinder><radius>0.05</radius><length>0.03</length></cylinder></geometry>
      </collision>
      <visual name="wr_vis">
        <geometry><cylinder><radius>0.05</radius><length>0.03</length></cylinder></geometry>
      </visual>
    </link>
    <joint name="right_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>wheel_right</child>
      <pose>0 -0.15 0.05 0 1.5708 0</pose>
      <axis><xyz>0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
    </joint>

    <plugin name="diff_drive" filename="gz-sim-diff-drive-system">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.30</wheel_separation>
      <wheel_radius>0.05</wheel_radius>
      <odom_frame>odom</odom_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <topic>/cmd_vel</topic>
      <max_linear_acceleration>5.0</max_linear_acceleration>
      <max_angular_acceleration>5.0</max_angular_acceleration>
    </plugin>
  </model>
</sdf>
```

---

## 2) Spawn the model
**Pick one method inside the container:**

- **GUI (keep Gazebo running in Terminal A):**  
  Gazebo → **Spawn** → **From file** → select  
  `/ws/ros2_ws/src/rover_description/models/tfr_bot.sdf`

- **CLI (Terminal A or a new Terminal B):**
  ```bash
  gz service -s /world/empty/create --reptype gz.msgs.EntityFactory \
    --req 'sdf_filename: "/ws/ros2_ws/src/rover_description/models/tfr_bot.sdf", name: "tfr_bot", pose: { position: {x: 0, y: 0, z: 0.1} }'
  ```

*(CLI can be in the same terminal if your Gazebo is running headless `-s` or you put the GUI in the background with `&`.)*

---

## 3) Start the ROS↔Gazebo bridge
- **Recommended:** run in a **separate Terminal B** so you can see logs:
  ```bash
  ros2 run ros_gz_bridge parameter_bridge
  ```
- **Same terminal option:** append `&` to run it in the background:
  ```bash
  ros2 run ros_gz_bridge parameter_bridge &
  ```

---

## 4) Create a tiny ROS 2 package and node
- Create package `tfr_rover_demo` (Python or C++) with deps: `rclpy|rclcpp`, `geometry_msgs`.
- Node publishes `geometry_msgs/Twist` on `/cmd_vel` at ~2 Hz:
  - `linear.x = 0.3`
  - `angular.z = 0.3`
- Build & source (**Terminal C** or same terminal if the bridge is backgrounded):
  ```bash
  cd /ws/ros2_ws
  rosdep install --from-paths src --ignore-src -r -y
  colcon build
  source install/setup.bash
  ```
- Run the node (**Terminal C** or background `&` in same shell):
  ```bash
  ros2 run tfr_rover_demo circle_driver
  ```

---

## 5) Verify
- **Any terminal:**
  ```bash
  ros2 topic echo /cmd_vel
  ros2 topic hz /cmd_vel
  ```
- In Gazebo, `tfr_bot` should move in a gentle circle.

---

## Files you must create
```
ros2_ws/
└─ src/
   ├─ rover_description/
   │  └─ models/
   │     └─ tfr_bot.sdf                 # minimal robot model (above)
   └─ tfr_rover_demo/                   # your package
      ├─ package.xml
      ├─ (setup.py / CMakeLists.txt)    # depends on Python or C++
      ├─ (setup.cfg / src/*.cpp)
      └─ tfr_rover_demo/
         └─ circle_driver.(py|cpp)      # node that publishes /cmd_vel
```

**Terminal tip:** keeping **Gazebo**, **bridge**, and **your node** in **separate terminals** is easiest to debug. If you prefer one terminal, you can background long-running processes with `&` and bring them back with `fg`.
