# Whole Body Control（WBC）学习实践仓库

## 项目简介
本仓库是笔者学习**全身控制（Whole Body Control, WBC）**时的学习笔记与实践工程，内容覆盖：
- WBC核心基础：雅可比矩阵、伪逆求解、任务优先级等理论的数学推导；
- 应用场景：WBC在人形机器人领域的典型落地方向；
- 实践练习：通过**3自由度机械臂**的任务实现，完成WBC入门实操，强化对基础知识的理解。


## 项目结构

whole_body_control/
├── config/ # 控制参数、机器人配置文件目录
├── include/ # C++ 头文件目录（若包含 C++ 代码）
├── launch/ # ROS 启动文件目录（一键启动机械臂 + 控制节点）
├── msg/ # 自定义 ROS 消息类型目录
├── scripts/ # WBC 核心控制逻辑（Python 脚本存放处，含学习笔记注释）
├── src/ # C++ 源代码目录（若包含 C++ 控制逻辑）
├── urdf/ # 3 自由度机械臂的 URDF 模型描述文件
├── CMakeLists.txt # ROS 功能包编译配置文件
├── package.xml # ROS 功能包信息配置（依赖、包名等）
└── README.md # 项目说明文档


## 环境依赖
运行项目需提前配置以下环境：
1. ROS版本：Noetic / Melodic（推荐Noetic，适配Python 3）
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
```bash
运行
cd ~/catkin_ws
catkin_make
source devel/setup.bash  
```

4.**启动 WBC 实践示例**

```bash
运行
roslaunch whole_body_control gazebo.launch```
