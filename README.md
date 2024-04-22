# mycobot320_moveit2

 ubuntu 22.04 
 
 ros2 humble
  
# [install moveit2](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html) 

need to test first

[binary install](https://moveit.ros.org/install-moveit2/source/) : for learning moveit

[Source build](https://moveit.ros.org/install-moveit2/source/)   : for creat packages

```bash
## make ros2 workspace
mkdir -p ~/[ros2 workspace dir]/src
sudo apt remove ros-$ROS_DISTRO-moveit*

## git moveit2
cd ~/[ros2 workspace dir]/src
git clone -b humble https://github.com/ros-planning/moveit2_tutorials.git
git clone -b humble https://github.com/ros-planning/moveit2.git
vcs import < moveit2_tutorials/moveit2_tutorials.repos
for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y


## colcon build
cd ~/[ros2 workspace dir]
colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
 ## if build dead, add  --parallel-workers 1




```

# git clone & build
```bash
cd ~/[ros2 workspace dir]/src
git clone https://github.com/cananella/mycobot320_moveit2.git
cp -r mycobot320_moveit2/* ./
rm -rf mycobot320_moveit2
cd ..
colcon build --packages-select mycobot_moveit mycobot_with_gripper
```
  
# rviz2 moveit control
```bash
ros2 launch mycobot_moveit demo.launch.py
```
![스크린샷 2024-04-15 14-39-02](https://github.com/cananella/mycobot320_moveit2/assets/97207725/513f9e2b-324d-4975-a475-a678b560ddee)

# whit gripper
```bash
ros2 launch mycobot_with_gripper demo.launch.py
```
![스크린샷 2024-04-16 17-17-56](https://github.com/cananella/mycobot320_moveit2/assets/97207725/d1460766-ea21-4d49-9203-a1ba5bb1340c)

 
### if error occurs [ERROR] [launch]: Caught exception in launch (see debug for traceback): 'capabilities'
```python
~/[ros2 workspace dir]/src/moveit2/moveit_configs_utils/moveit_configs_utils/launches.py line:203
change: default_value=moveit_config.move_group_capabilities["capabilities"],
-> default_value=moveit_config.move_group_capabilities,
```
 
# [control whit code](https://moveit.picknik.ai/main/doc/tutorials/your_first_project/your_first_project.html)

```bash
cd ~/[ros2 workspace dir]/src

ros2 pkg create \
 --build-type ament_cmake \
 --dependencies moveit_ros_planning_interface rclcpp \
 --node-name hello_moveit hello_moveit

gedit helo_moveit/src/hello_moveit.cpp
```
 
hello_moveit.cpp
```c++ 
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "manipulator"); //<- need to input controller name
  
  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;               // <- need to change pose whthin mycobot can move
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);
  
  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
  
  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
  
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
```
