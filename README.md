# Awesome Quadruped Locomotion 🐕‍🦺  
四足狗（Go2 / Go1 / A1 / Solo12 …）学习与实战总览

> 目标：帮你从 **“装好仿真环境” → “跑出 RL 策略” → “部署到真机四足狗”** 走完一条闭环路径。  
> 重点聚焦：**Unitree Go2 / Go1 系列**，同时兼顾 ODRI Solo12 等开源平台。

---

## 目录

- [0. 适用对象 & 使用方式](#0-适用对象--使用方式)
- [1. 通用训练框架 / 环境（腿足与人形共用）](#1-通用训练框架--环境腿足与人形共用)
- [2. 厂商 / 平台套件（面向 Go2 / H1 等）](#2-厂商--平台套件面向-go2--h1-等)
- [3. 四足机器人按目标场景汇总](#3-四足机器人按目标场景汇总)
  - [3A. 速度 / 姿态跟踪（平地 / 粗糙地形）](#3a-速度--姿态跟踪平地--粗糙地形)
  - [3B. 越障 / 跑酷（敏捷动作与安全约束）](#3b-越障--跑酷敏捷动作与安全约束)
  - [3C. 视觉导航（Exteroceptive Locomotion）](#3c-视觉导航exteroceptive-locomotion)
  - [3D. 起身恢复 / 跌倒恢复](#3d-起身恢复--跌倒恢复)
  - [3E. ODRI Solo12 / Bolt 等开源平台](#3e-odri-solo12--bolt-等开源平台)
- [4. 部署与工程化（从仿真到真机）](#4-部署与工程化从仿真到真机)
- [5. 针对不同硬件的“快速上手清单”](#5-针对不同硬件的快速上手清单)
  - [5.1 Go2 / Go1 系列](#51-go2--go1-系列)
  - [5.2 ODRI Solo12](#52-odri-solo12)
  - [5.3 ODRI Bolt（小型双足）](#53-odri-bolt小型双足)
- [6. 更多参考 / 综述清单](#6-更多参考--综述清单)

---

## 0. 适用对象 & 使用方式

**适合谁？**

- 有 / 准备有：Go2 / Go1 / A1 / Solo12 等四足平台
- 想要：
  - 在 **Isaac Gym / Isaac Lab / Legged Gym** 里训练四足行走策略
  - 做 **越障 / 跑酷 / 视觉导航 / 起身恢复** 等高级技能
  - 最终把策略 **部署到真机** 上跑起来

**怎么用这个 README？**

- 当作一个 **带标签的资源索引**：
  - 想做“平地&粗糙地形速度跟踪” → 看 [3A](#3a-速度--姿态跟踪平地--粗糙地形)
  - 想做“跑酷/越障” → 看 [3B](#3b-越障--跑酷敏捷动作与安全约束)
  - 想做“视觉导航 + 行走” → 看 [3C](#3c-视觉导航exteroceptive-locomotion)
  - 想做“起身恢复” → 看 [3D](#3d-起身恢复--跌倒恢复)
- 如果你是 **Go2/H1 直接用户**，可以重点参考 [2](#2-厂商--平台套件面向-go2--h1-等) + [5.1](#51-go2--go1-系列)。

---

## 1. 通用训练框架 / 环境（腿足与人形共用）

> 这些是“**所有四足/人形 RL 项目几乎都会碰一下的基础设施**”。

- **Legged Gym（ETH RSL）**  
  - 四足 / 多足 RL 环境，基于 Isaac Gym，带粗糙地形、域随机化、推挤扰动等；大量论文直接以此为基线。  
  - 代码：  
    https://github.com/leggedrobotics/legged_gym  

- **Isaac Gym Envs / Isaac Lab（官方统一框架）**  
  - Isaac Gym 示例环境 + Isaac Lab（后续统一框架）：  
    - 原生支持人形 / 四足任务  
    - 官方集成 RSL-RL、skrl、RL-Games 等训练脚本  
  - 链接：  
    - Envs：https://github.com/isaac-sim/IsaacGymEnvs  
    - Isaac Lab 文档：https://isaac-sim.github.io/IsaacLab/  
    - Isaac Lab 仓库：https://github.com/isaac-sim/IsaacLab  

- **RSL-RL**（ETH RSL 高效 PPO 库，GPU 端到端）  
  - 基本是 Legged Gym / Isaac 系列的“标配 RL 引擎”。  
  - 代码：  
    https://github.com/leggedrobotics/rsl_rl  

- **skrl**（面向 Isaac Gym / Isaac Lab / Gymnasium 的 PyTorch RL 库）  
  - 示例丰富、入门友好，官方文档里有专门的 Isaac Lab 集成。  
  - 文档与示例：  
    https://skrl.readthedocs.io/en/latest/intro/examples.html  
  - Isaac Lab 集成说明：  
    https://skrl.readthedocs.io/en/latest/api/envs/isaaclab.html  

---

## 2. 厂商 / 平台套件（面向 Go2 / H1 等）

> 如果你用的是 **Unitree 系列（Go2 / H1 / G1）**，可以优先踩这里，少走弯路。

- **Unitree RL Gym**  
  - 面向 Go2 / H1 / H1_2 / G1 的 RL 实现，支持 Isaac Gym、MuJoCo，附带物理部署链路。  
  - 仓库：  
    https://github.com/unitreerobotics/unitree_rl_gym  
  - Unitree 开源汇总页：  
    https://www.unitree.com/mobile/opensource  

- **Unitree RL Lab**  
  - 基于 Isaac Lab 的官方 RL 环境集：Go2、H1、G1-29DoF 等，便于和 Isaac 生态打通。  
  - 仓库：  
    https://github.com/unitreerobotics/unitree_rl_lab  

- **Unitree Sim IsaacLab**  
  - Unitree 机器人在 Isaac Lab 的模拟 / 数据采集与验证仓。  
  - 和真实机 DDS 协议一致，Sim→Real 迁移会更顺。  
  - 仓库：  
    https://github.com/unitreerobotics/unitree_sim_isaaclab  

---

## 3. 四足机器人按目标场景汇总

### 3A. 速度 / 姿态跟踪（平地 / 粗糙地形）

> 最典型的“跑起来就行”的 RL 任务——平地/坡地/随机地形速度跟踪。

- **Legged Gym 基线任务**  
  - 支持平地速度跟踪、粗糙地形行走（域随机化、噪声、推挤），Sim2Real 已被反复验证。  
  - 仓库：  
    https://github.com/leggedrobotics/legged_gym  

- **Walk These Ways（MoB，多行为泛化，Go1）**  
  - 在 Go1 上用 PPO + 多行为（MoB）学多种 gait，内含部署到 Go1 的 SDK 桥接。  
  - 仓库：  
    https://github.com/Improbable-AI/walk-these-ways  

- **skrl / Isaac Lab 速度跟踪任务示例**  
  - A1 / ANYmal 等四足的速度跟踪任务示例与训练脚本。  
  - 文档：  
    https://skrl.readthedocs.io/en/latest/intro/examples.html  
  - Isaac Lab 可用环境列表：  
    https://isaac-sim.github.io/IsaacLab/main/source/overview/environments.html  

---

### 3B. 越障 / 跑酷（敏捷动作与安全约束）

> 当你不满足于“在平地跑”，而是想做 **越障、台阶、跑酷 + 安全约束**。

- **ABS（Agile-But-Safe，RSS 2024）**  
  - 针对高速无碰撞越障的约束式 RL（基于 RSL-RL / Legged Gym 生态）。  
  - 仓库：  
    https://github.com/LeCAR-Lab/ABS  

- **Robot Parkour Learning / 四足跑酷方向**  
  - 跑酷学习系统（论文 + 示例），强调多样、视觉引导的敏捷技能。  
  - 总览页：  
    https://robot-parkour.github.io/  
  - 论文 PDF（PMLR）：  
    https://proceedings.mlr.press/v229/zhuang23a/zhuang23a.pdf  
  - Science Robotics 版的敏捷导航：  
    https://www.science.org/doi/10.1126/scirobotics.adi7566  

- **ViNL（Visual Navigation & Locomotion over Obstacles, ICRA 2023）**  
  - 把视觉导航（Habitat 训练）与越障行走（Isaac 训练）拼接，给出端到端示例。  
  - 仓库：  
    https://github.com/SimarKareer/ViNL  

---

### 3C. 视觉导航（Exteroceptive Locomotion）

> 不再“闭着眼走路”，而是结合 **高度图 / 点云 / RGB 视觉** 做外感知行走。

- **Robust Perceptive Locomotion（Science Robotics 2022）**  
  - 融合外感知（高度图 / 点云）与本体感知的鲁棒行走管线，ANYmal 野外徒步示范。  
  - 项目页：  
    https://leggedrobotics.github.io/rl-perceptiveloco/  

- **面向低成本平台的外感知 RL 行走（2025）**  
  - 同时训练策略与状态估计器，支持实时高程图构建。  
  - 论文：  
    https://arxiv.org/html/2505.12537v1  

- **Go1 视觉 / 导航实用栈（ROS2 Nav2）**  
  - 非 RL，但可以与学到的行走策略对接，用于全栈导航。  
  - 仓库：  
    https://github.com/ngmor/unitree_nav  

---

### 3D. 起身恢复 / 跌倒恢复

> 真机一定会摔，**“站起来”** 是工业落地前绕不过去的一环。

- **四足恢复策略（2019 经典工作）**  
  - 分层 RL 学到的恢复动作（行为选择 + 专项策略）。  
  - 论文：  
    https://arxiv.org/abs/1901.07517  

- **RL + MPC 混合控制（含 Recovery Stand）**  
  - 示例仓含 “Recovery Stand” 等状态切换（MPC / RL 混编）。  
  - 仓库：  
    https://github.com/silvery107/rl-mpc-locomotion  

---

### 3E. ODRI Solo12 / Bolt 等开源平台

> 如果你想用 **开源硬件平台（Solo12 / Bolt）** 做研究或教学，这部分很有用。

- **Solo12 深度 RL 控制（Science Reports）**  
  - 速度跟踪、课程学习与域随机化全流程，论文与代码一一对应。  
  - 论文：  
    https://www.nature.com/articles/s41598-023-38259-7  
  - 代码：  
    https://github.com/Gepetto/soloRL  

- **Bolt（ODRI 双足）行走 RL 学生项目实现**  
  - Bolt 行走的 DRL 训练与报告；Bolt 驱动与 URDF 由 ODRI 提供。  
  - 训练仓：  
    https://github.com/rafacelente/bolt  
  - Bolt 驱动：  
    https://github.com/open-dynamic-robot-initiative/bolt  
  - ODRI 官网：  
    https://open-dynamic-robot-initiative.github.io/  

---

## 4. 部署与工程化（从仿真到真机）

> 训练只是第一步，**“真机跑起来”** 更关键。

- **Unitree SDK / 示例**  
  - Go1 / Go2 / H1 / G1 的 SDK 与示例，多为 ROS2 / 高层控制接口。  
  - 官方开源页汇总：  
    https://www.unitree.com/mobile/opensource  
  - Walk-These-Ways 中有 `unitree_legged_sdk` 的部署示例：  
    https://github.com/Improbable-AI/walk-these-ways  

- **Isaac Sim / Isaac Lab 策略示例运行**  
  - 官方提供 H1 与 Spot 的策略示例加载，用来验证策略和调试传感器 / 时序非常方便。  
  - Policy 示例文档：  
    https://docs.isaacsim.omniverse.nvidia.com/4.5.0/robot_simulation/ext_isaacsim_robot_policy_example.html  
  - Isaac Lab 教程与 Showroom：  
    https://isaac-sim.github.io/IsaacLab/  
    https://isaac-sim.github.io/IsaacLab/main/source/overview/showroom.html  

---

## 5. 针对不同硬件的“快速上手清单”

### 5.1 Go2 / Go1 系列 ✅（重点）

**1）速度控制（平地 / 粗糙）：**

- 选项 A：直接用 Unitree 官方套件
  - Unitree RL Lab（Isaac Lab）  
    https://github.com/unitreerobotics/unitree_rl_lab  
  - Unitree RL Gym（Isaac Gym / MuJoCo）  
    https://github.com/unitreerobotics/unitree_rl_gym  

- 选项 B：Legged Gym + RSL-RL 自己练
  - https://github.com/leggedrobotics/legged_gym  

**2）越障 / 跑酷：**

- 约束安全越障：  
  - ABS（Agile-But-Safe）：  
    https://github.com/LeCAR-Lab/ABS  
- 视觉 + 越障联合：  
  - ViNL：  
    https://github.com/SimarKareer/ViNL  
- 多行为跑酷对标：  
  - Walk-These-Ways：  
    https://github.com/Improbable-AI/walk-these-ways  

**3）视觉导航：**

- RL 端到端方案：ViNL（Habitat + Isaac）  
  - https://github.com/SimarKareer/ViNL  
- 工程实用栈：Go1 / Go2 的 ROS2 Nav2  
  - https://github.com/ngmor/unitree_nav  

**4）起身恢复：**

- 方法参考：  
  - 经典恢复论文：https://arxiv.org/abs/1901.07517  
  - RL+MPC 示例：https://github.com/silvery107/rl-mpc-locomotion  
- 落地思路：  
  - 单独训练一个 Recovery Policy  
  - 在高层策略或状态机里做 **“摔倒检测 → 切换到恢复策略 → 回到行走策略”** 的行为选择

---

### 5.2 ODRI Solo12 ✅

**1）速度控制：**

- 直接复现实验用 `soloRL` 环境：  
  - 论文：https://www.nature.com/articles/s41598-023-38259-7  
  - 代码：https://github.com/Gepetto/soloRL  

**2）越障 / 视觉扩展：**

- 可参考 ViNL / ABS 的任务设计，迁移到 Solo12 仿真模型（MuJoCo / Isaac 皆可）：  
  - ViNL：https://github.com/SimarKareer/ViNL  
  - ABS：https://github.com/LeCAR-Lab/ABS  

---

### 5.3 ODRI Bolt（小型双足）

> 虽然是双足，但很多训练框架和思路与四足共用，适合教学和研究。

- 行走训练流程：  
  - 学生项目仓：  
    https://github.com/rafacelente/bolt  
- 硬件与 URDF 支持：  
  - Bolt 驱动：  
    https://github.com/open-dynamic-robot-initiative/bolt  

---

## 6. 更多参考 / 综述清单

> 当你需要写综述 / PPT / 立项报告，可以直接从这里展开。

- **HumanoidBench（27 个全身任务）**  
  - 虽然侧重人形，但有助于理解 **多任务评测框架**。  
  - 站点：https://humanoid-bench.github.io/  
  - 论文（RSS 2024）：https://arxiv.org/pdf/2403.10506  

- **Awesome 系列清单（Humanoid / Legged / Isaac Gym）**  
  - Humanoid Learning：  
    https://github.com/jonyzhang2023/awesome-humanoid-learning  
  - Legged Locomotion Learning：  
    https://github.com/gaiyi7788/awesome-legged-locomotion-learning  
  - Isaac Gym 资源：  
    https://github.com/robotlearning123/awesome-isaac-gym  




