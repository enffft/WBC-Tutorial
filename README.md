# Whole Body Control（WBC）学习实践仓库

## 项目简介
本仓库是笔者学习 **全身控制（Whole Body Control, WBC）** 时所整理的学习笔记与实践示例工程。内容包含：

- **WBC 基础理论**：雅可比矩阵、伪逆求解、任务优先级、QP/约束处理等核心数学推导；
- **典型应用场景**：WBC 在人形机器人等多任务系统中的落地方向；
- **入门实践**：基于 **3 自由度机械臂** 的 WBC 教程，通过实际任务强化对理论的理解。

本仓库内包含两份核心资料：

- **WBC学习笔记.pdf**：系统介绍 WBC 理论基础；
- **WBC三自由度机械臂教程.pdf**：通过一支三自由度机械臂给出 WBC 控制器构建步骤、任务空间定义方法，并在文末提供参考答案。

> ⚠ 注意：示例工程 **尚未完成完整的 WBC 控制器实现**。  
> 学习者需要根据教程内容，**自行填补 `scripts/` 目录下的 WBC 控制核心函数**。  
> 参考答案见《WBC三自由度机械臂教程.pdf》的最后部分。

---

## 项目结构

```
whole_body_control/
├── config/                            # rviz配置文件
├── include/                           #
├── launch/                            # ROS 启动文件目录（一键启动机械臂 + 控制节点）
├── msg/                               # 自定义 ROS 消息类型目录
├── scripts/                           # WBC 核心控制逻辑
├── src/                               # 
├── WBC学习笔记.pdf                     # 
├── WBC三自由度机械臂教程.pdf            # 
├── urdf/                              # 3 自由度机械臂的 URDF 模型描述文件
├── CMakeLists.txt                     # ROS 功能包编译配置文件
├── package.xml                        # ROS 功能包信息配置
└── README.md                          # 项目说明文档
```

## 环境依赖
运行项目需提前配置以下环境：
1. ROS版本：ubuntu 20.04 Noetic 
2. 依赖库：
   - Python 3
   - NumPy（安装命令：`pip install numpy`）
3. 可视化工具：RViz（随ROS默认安装）


## 使用步骤
1. **创建/进入ROS工作空间**
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

2. **克隆仓库**
```bash
git clone https://github.com/enffft/WBC-Tutorial.git
```
3.**编译工作空间**
运行
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash  
```

4.**启动 WBC 实践示例**
运行
```bash
roslaunch whole_body_control gazebo.launch
```
