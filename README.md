以下是规范化后的 README 文件：

---

# 编译指南

你好！这是关于大模型规划相关的代码，为了确保代码能够正常运行，请按照以下步骤操作：

## 步骤 1: 克隆代码并安装 ROS 依赖

首先，将代码克隆到一个工作空间目录，并下载所有必要的 ROS 依赖：

```bash
mkdir -p catkin_workspace/src
cd catkin_workspace/src
git clone -b https://github.com/CumulusAlpha/my_kinova.git
cd ../
rosdep install --from-paths src --ignore-src -y
```

## 步骤 2: 安装 MoveIt

执行以下命令安装 MoveIt：

```bash
sudo apt install ros-noetic-moveit
```

## 步骤 3: 编译工作空间并加载环境

编译工作空间，并加载环境变量：

```bash
catkin_make
source devel/setup.bash
```

> **注意**：如果编译没有报错，则表示成功。但某些电脑上可能会缺少 `moveit-visual-tools`，可通过以下命令单独安装：

```bash
sudo apt install ros-noetic-moveit-visual-tools
```

---

# Demos

以下是两个主要的 Demo 使用指南：

## 1. Mujoco 环境中的机器人可视化

运行以下命令启动机器人可视化工具：

```bash
rosrun kinova_mujoco mujoco_refresher.py
```

## 2. 使用 BehaviorTree 控制机械臂抓取物体

### 第一步：在第一个终端加载机器人描述

```bash
roslaunch my_own_robot demo.launch # 加载机器人描述
```

### 第二步：在第二个终端运行抓取任务

```bash
rosrun kinova_mujoco mujoco_behaviortree
```

---

通过上述步骤，你应该能够成功运行项目代码和示例。若有任何问题，请随时联系！
English Version
---
# Compilation

Hello, this is the code I developed related to large model planning. To ensure it runs correctly, please follow the steps below:

### Step 1: Clone all the code into a workspace directory and download all ROS dependencies:

```bash
mkdir -p catkin_workspace/src
cd catkin_workspace/src
git clone -b https://github.com/CumulusAlpha/my_kinova.git
cd ../
rosdep install --from-paths src --ignore-src -y
```

### Step 2: Install MoveIt

```bash
sudo apt install ros-noetic-moveit
```

### Step 3: Compile your workspace and source it

```bash
catkin_make
source devel/setup.bash
```

If there are no errors, the compilation was successful. However, on some computers, `moveit-visual-tools` might not be installed. In such cases, please use the following command to install it:

```bash
sudo apt install ros-noetic-moveit-visual-tools
```

---

# Demos

### Robot Visualization in Mujoco Environment:

```bash
rosrun kinova_mujoco mujoco_refresher.py
```

### Using BehaviorTree to Control the Robotic Arm for Grasping Objects:

1. In the first terminal, run:

```bash
roslaunch my_own_robot demo.launch # load description of robots
```

2. In the second terminal, run:

```bash
rosrun kinova_mujoco mujoco_behaviortree
```


