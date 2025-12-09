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
- [7. 实操小贴士：把“研究仓”变成“工程产线”](#7-实操小贴士把研究仓变成工程产线)

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
  - 在 Go1
