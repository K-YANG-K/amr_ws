# Nav2 设计思想与 ROS2 技术体系总览

------

# 一、Nav2 是什么（本质定位）

Nav2 本质不是一个算法包，而是一个 **机器人导航操作系统（Navigation Framework）**。

目标只有一句话：

👉 把 “去某个地方” 变成一个 **可扩用、可插拔、可工业化运行** 的系统。

所以 Nav2 解决的是：

| 层级   | 解决问题                     |
| ------ | ---------------------------- |
| 算法层 | 路径规划、避障、控制         |
| 软件层 | 多节点协作、异步执行         |
| 工程层 | 生命周期、故障恢复、扩展插件 |
| 系统层 | 可组合部署、实时性、稳定性   |

这就是 Nav2 的真正高度。

------

# 二、Nav2 总体架构（宏观）

Nav2 的设计核心是 **Task → Behavior Tree → Servers → Plugins**

整体流程：

```
用户目标 → BT导航任务树 → 各类Server → 插件算法执行 → 控制机器人
```

核心节点：

| 节点              | 作用               |
| ----------------- | ------------------ |
| bt_navigator      | 导航大脑（行为树） |
| planner_server    | 路径规划           |
| controller_server | 路径跟踪           |
| smoother_server   | 路径平滑           |
| behavior_server   | 恢复行为           |
| map_server        | 提供地图           |
| amcl / slam       | 定位               |
| lifecycle_manager | 生命周期管理       |

------

# 三、Nav2 最核心设计思想

这一部分是灵魂。

------

## 1️⃣ 行为树 = 导航大脑

Nav2 最大的革命：
 用 **Behavior Tree** 代替传统状态机。

为什么不用状态机？

状态机问题：

- 状态爆炸
- 不可扩展
- 无法并行
- 恢复逻辑难写

行为树优势：

| 能力   | 说明             |
| ------ | ---------------- |
| 模块化 | 每个节点独立     |
| 可恢复 | 支持fallback     |
| 可并行 | 支持并行节点     |
| 可组合 | XML 即可重构任务 |
| 可扩展 | 插件新增行为     |

Nav2 的导航本质：

```
while(true):
    tick 整棵行为树
```

这就是你之前理解的：

> Action 回调里 while tick BT

完全正确 👍

------

## 2️⃣ Task Oriented Navigation（任务式导航）

Nav2 不再是：

```
算路径 → 跟踪 → 结束
```

而是：

```
完成一个任务（Task）
```

导航被看作：

- 一个长期运行的 Action
- 内部不断 replan / recontrol / recovery

所以导航变成：

```
NavigateToPose.action
    ↳ 内部执行 BT
```

这就是为什么 nav2 用 **Action 而不是 Service**。

------

## 3️⃣ 全插件化架构（Plugin First）

Nav2 的所有算法都不是写死的：

| 功能       | 插件接口                 |
| ---------- | ------------------------ |
| 全局规划   | nav2_core::GlobalPlanner |
| 局部控制   | nav2_core::Controller    |
| 恢复行为   | nav2_core::Recovery      |
| 代价地图层 | nav2_costmap_2d::Layer   |

这意味着：

👉 Nav2 是一个导航 **平台**
 而不是一个导航 **算法**

你可以随便替换算法。

------

## 4️⃣ 分层 Costmap 思想

Costmap = 世界模型

采用 **Layered Costmap**

像 PS 一样叠图层：

```
static layer      静态地图
obstacle layer    传感器障碍
inflation layer   膨胀
voxel layer       3D障碍
```

设计思想：

👉 环境 = 多信息源融合

这就是现代机器人感知的思想。

------

## 5️⃣ 恢复行为（Recovery System）

导航不可能永远成功。

Nav2 内置失败哲学：

```
失败不是异常，是常态
```

典型恢复：

- 清除costmap
- 原地旋转
- 后退
- 重新规划

行为树实现：

```
Fallback
 ├── FollowPath
 ├── ClearCostmap
 └── Spin
```

这就是工业级鲁棒性设计。

------

# 四、Nav2 用到的 ROS2 核心知识（重点）

这部分是 ROS2 技术精华。

------

## 1️⃣ Lifecycle Node（生命周期节点）

Nav2 所有服务器都是 lifecycle node。

状态机：

```
unconfigured → inactive → active → shutdown
```

为什么必须用 lifecycle？

机器人启动需要 **严格顺序**：

```
先地图 → 再定位 → 再costmap → 再规划 → 再控制
```

由 lifecycle_manager 统一管理。

这就是：

👉 系统级启动管理

------

## 2️⃣ Action 通信机制

导航是长期任务：

必须支持：

- 可取消
- 有反馈
- 可中断

所以使用：

```
NavigateToPose.action
```

Action 在 Nav2 中的地位：

| 功能         | 使用位置                |
| ------------ | ----------------------- |
| 用户 → Nav2  | 发送导航目标            |
| BT → Servers | 调 planner / controller |

Nav2 内部是 **Action 套 Action**。

------

## 3️⃣ Executor + CallbackGroup（关键）

这是 Nav2 能不卡死的关键。

问题：
 BT tick 是 while 循环
 如果在同一个 executor → 会阻塞所有回调

Nav2 解决：

每个 server 都有：

```
隐藏 node + 独立 executor + callback group
```

实现：

- BT 可以 while tick
- 同时 planner / controller 还能处理请求

这就是你之前理解的那段 👍

------

## 4️⃣ TF2 变换体系

导航本质依赖三大坐标系：

```
map → odom → base_link
```

Nav2 到处传 tf_buffer：

因为：

- planner 需要 global pose
- controller 需要 local pose
- costmap 需要 sensor tf

所以 BT blackboard 中必须存 tf_buffer。

------

## 5️⃣ Component 组件化部署

Nav2 使用：

```
rclcpp_components
```

所有 server 不是独立进程，而是：

👉 同一进程的多个组件

好处：

| 优势       | 说明     |
| ---------- | -------- |
| 零拷贝通信 | 性能高   |
| 部署灵活   | 可拆可合 |
| 资源节省   | 工业必备 |

------

## 6️⃣ 参数系统 + YAML 配置

Nav2 几乎所有行为都参数化：

- 规划器选择
- 控制器选择
- costmap配置
- BT XML路径

实现：

```
declare_parameter()
```

这就是 ROS2 参数驱动架构。

------

# 五、Nav2 导航完整执行流程

完整一次导航：

------

### Step 1 用户发目标

```
ros2 action send_goal /navigate_to_pose
```

------

### Step 2 BT Navigator 启动行为树

```
NavigateToPose.xml
```

------

### Step 3 规划路径

```
ComputePathToPose → planner_server
```

------

### Step 4 跟踪路径（循环）

```
FollowPath → controller_server
```

循环执行：

```
replan → control → check → replan → control
```

------

### Step 5 失败 → 恢复行为

```
ClearCostmap → Spin → Replan
```

------

### Step 6 到达目标 → 返回 SUCCESS

------

# 六、Nav2 的终极设计哲学

总结为 6 个关键词：

| 思想     | 含义       |
| -------- | ---------- |
| 行为树   | 决策逻辑   |
| Action化 | 任务执行   |
| 插件化   | 算法扩展   |
| 生命周期 | 系统管理   |
| 组件化   | 高性能部署 |
| 恢复机制 | 工业鲁棒性 |

------

# 七、回答你最初的问题（升华）

### 动态感知预测 Nav2 能做吗？

答案现在非常清晰：

Nav2 是 **导航框架**
 不是 **感知预测框架**

但它 **预留了接口**：

你可以接入：

- 动态障碍预测 → costmap layer 插件
- 动态轨迹预测 → controller 插件
- 社会导航 → 新 planner 插件

所以：

👉 Nav2 = 可扩展导航操作系统

------

如果你愿意，我可以下一步帮你画一张
 **Nav2 架构图（进程 + 线程 + 通信）**
 你会彻底通透。

可以

太好了，这一步我们把 **Nav2 真正“画出来”**。
 这张图会包含：**进程、线程、通信、BT、Action、TF、Costmap 全部串起来**。

我用“分层架构图”的方式讲，你可以脑中把它想成一张系统拓扑图。

------

# Nav2 系统级架构图（进程 / 线程 / 通信）

我们从 **最外层 → 最核心** 一层一层往里走。

------

# 第一层：用户层（User Layer）

这是机器人“外部世界”。

```
RViz / App / Web / Fleet Manager
            │
            ▼
   NavigateToPose Action
```

用户只做一件事：

```
发送导航目标
```

ROS2接口：

```
/navigate_to_pose  (Action Client)
```

------

# 第二层：Nav2 主进程（Component Container）

这一层是重点。

Nav2 默认运行在 **一个进程中**：

```
nav2_container  (单进程)
```

但内部包含很多 **组件节点（Components）**：

```
┌──────────────────────────────┐
│         nav2_container        │
│                               │
│  bt_navigator                 │
│  planner_server               │
│  controller_server            │
│  behavior_server              │
│  smoother_server              │
│  map_server                   │
│  lifecycle_manager            │
└──────────────────────────────┘
```

关键设计：

👉 单进程 + 多组件
 👉 零拷贝通信
 👉 工业级性能

------

# 第三层：导航大脑（BT Navigator）

这是 Nav2 的 **CPU**。

```
          NavigateToPose Action Server
                        │
                        ▼
                BT Navigator
```

内部发生的事：

```
Action 回调触发
      ↓
while(rclcpp::ok())
      ↓
tick 整棵行为树
```

行为树核心循环：

```
┌────────────────────────────┐
│   NavigateToPose.xml        │
│                            │
│  ComputePathToPose         │
│  FollowPath                │
│  Recovery Behaviors        │
└────────────────────────────┘
```

BT 只做一件事：

👉 **调度各个 Server**

------

# 第四层：Task Servers（功能服务器层）

行为树不会算路径。
 它只会调用 Server。

```
BT Navigator
     │
     ├── planner_server
     ├── controller_server
     ├── smoother_server
     └── behavior_server
```

通信方式：

```
BT  →  Servers  使用 Action
```

注意：

Nav2 内部是 **Action 套 Action**。

------

# 第五层：线程模型（最重要🔥）

这是 Nav2 最精妙的设计。

如果 BT 在 while 循环里执行，理论上会阻塞整个 ROS。

Nav2 的解决方案：

## 每个 Server 都有独立 Executor

```
bt_navigator
    └── executor (线程1)

planner_server
    └── executor (线程2)

controller_server
    └── executor (线程3)

behavior_server
    └── executor (线程4)
```

这意味着：

```
BT while tick 不会卡死系统
```

同时：

- 规划可以算路径
- 控制器可以算速度
- costmap 可以更新

全部并行运行。

这就是 Nav2 的 **并发架构核心**。

------

# 第六层：Costmap（世界模型层）

所有导航 Server 都依赖 Costmap。

```
        costmap_2d
            │
 ┌──────────┼──────────┐
 │          │          │
global   local     voxel
costmap  costmap   layer
```

Costmap 数据来源：

```
Map Server → 静态地图
Lidar      → 障碍物
TF         → 机器人位置
```

这一层解决：

👉 机器人周围世界长什么样

------

# 第七层：TF 坐标变换层

导航离不开 TF。

核心坐标树：

```
map → odom → base_link → sensors
```

所有 Server 都会不断查询：

```
tf_buffer->lookupTransform()
```

用途：

| Server     | 用 TF 做什么     |
| ---------- | ---------------- |
| planner    | 获取 global pose |
| controller | 获取 local pose  |
| costmap    | 转换传感器数据   |

------

# 第八层：算法插件层（最底层）

真正干活的其实是插件。

```
planner_server
    ├── NavFn
    ├── SmacPlanner
    └── 自定义Planner

controller_server
    ├── DWB
    ├── RPP
    └── MPC
```

Nav2 只是框架。

算法全部可替换。

------

# 完整导航时序图（最关键）

现在把全部串起来：

```
用户发送目标
      ↓
NavigateToPose Action
      ↓
BT Navigator 开始 tick
      ↓
调用 planner_server (Action)
      ↓
生成全局路径
      ↓
循环：
    调 controller_server
    输出速度
    更新 costmap
    检查是否到达
      ↓
失败 → 调 recovery
成功 → 返回 Result
```

------

# Nav2 架构一句话总结

```
用户目标
   ↓
行为树决策
   ↓
Action 调度 Servers
   ↓
Servers 调度插件
   ↓
插件使用 Costmap + TF
   ↓
输出速度控制机器人
```

------

# 你现在已经达到的理解层级

你现在已经可以理解：

- Nav2 为什么不会被 BT 卡死
- 为什么要 lifecycle
- 为什么必须 action
- 为什么要组件化
- 为什么一切都插件化

你已经进入 **Nav2 架构级理解**了。







你问的是**真正工业级的插件分层问题**：

> 两种车型：差速 + 阿克曼
>  都想用 **TEB**
>  但必须 **改TEB代码本身**
>  该怎么设计工程，才能不把TEB改成一坨 if(vehicle_type)？

这问题非常专业，而且很多团队真的踩坑。

我们一步一步讲清楚。

------

# 先说核心结论 ⭐⭐⭐

如果 **同一个算法需要因车型改代码**，正确设计不是：

❌ 改 Nav2 controller
 ❌ 写两个 controller
 ❌ 在 TEB 里写 if(vehicle_type)

而是：

> **把 TEB 拆成 “通用优化器 + 车型运动学插件”**

这是关键。

------

# 一、为什么 TEB 会被车型“污染”

TEB本质在干嘛？

它在解一个优化问题：

```
min cost(轨迹)
s.t.
    运动学约束
    障碍物约束
    速度约束
    曲率约束
```

关键来了：

👉 **唯一与车型强相关的，是“运动学约束”**

其它全部是共用的：

| 约束       | 是否车型相关 |
| ---------- | ------------ |
| 障碍物距离 | ❌            |
| 时间最优   | ❌            |
| 平滑       | ❌            |
| 速度限制   | ❌            |
| 加速度限制 | ❌            |
| 非完整约束 | ⚠️            |
| 转弯半径   | ✅            |
| 转向角     | ✅            |
| 侧滑       | ✅            |

真正需要改的只有：

> **Kinematics Model（运动学模型）**

这就是突破口。

------

# 二、TEB源码其实已经半支持这种设计

TEB里其实有一个类：

```
BaseRobotFootprintModel
RobotFootprintModel
```

但 **没有真正抽象运动学模型**

所以很多公司会 fork TEB。

我们要做的是：

> **给 TEB 增加一个 KinematicModel 插件层**

------

# 三、正确的工业分层（关键图）

现在 TEB 应该被拆成三层：

```
Nav2 Controller Plugin
        │
        ▼
TEB Optimizer（通用）
        │
        ▼
Kinematic Model Plugin（按车型）
```

这才是正确设计。

不是写两个 TEB。

------

# 四、代码该如何拆 ⭐（重点）

## 第一步：定义运动学接口

创建一个接口：

```
class KinematicModel
{
public:
  virtual void AddKinematicEdges(
        teb::Graph& graph,
        teb::Trajectory& traj) = 0;

  virtual double getMinTurningRadius() = 0;

  virtual bool isHolonomic() = 0;
};
```

注意：

TEB本质是 g2o graph。

我们让不同车型：
 👉 往图里 **添加不同约束边**

这一步非常关键。

------

## 第二步：实现不同车型模型

### 差速模型

```
class DiffDriveModel : public KinematicModel
{
  void AddKinematicEdges(...) override
  {
      // 非完整约束
      // 禁止侧向速度
      addEdgeNonHolonomic();
  }
};
```

### 阿克曼模型

```
class AckermannModel : public KinematicModel
{
  void AddKinematicEdges(...) override
  {
      // 最小转弯半径约束
      addEdgeKinematicsCarlike();
  }
};
```

### 麦轮模型（未来）

```
class MecanumModel : public KinematicModel
{
  void AddKinematicEdges(...) override
  {
      // 无非完整约束
      // 全向模型
  }
};
```

TEB优化器完全不用改。

------

# 五、TEB 主优化器只保留一份

核心优化流程：

```
void TebOptimizer::buildGraph()
{
    addObstacleEdges();
    addVelocityEdges();
    addTimeOptimalEdges();

    // ⭐关键：交给车型插件
    kinematic_model_->AddKinematicEdges(graph, traj);
}
```

就这一行。

算法干净了。

------

# 六、如何在 Nav2 里配置不同车型

现在 controller 仍然只有 **一个**：

```
TebController
```

但它加载不同 KinematicModel：

```
teb_controller:
  ros__parameters:
    kinematic_model: "ackermann"
```

然后用 pluginlib：

```
kinematic_model_loader_.createSharedInstance(type);
```

实现：

```
teb_kinematics/DiffDriveModel
teb_kinematics/AckermannModel
```

------

# 七、最终工程结构 ⭐⭐⭐

这就是你真正要的结构：

```
nav2_teb_controller/
│
├── teb_core/                 ← 通用TEB优化器（永远不改）
│
├── teb_kinematics/           ← ⭐车型插件
│   ├── diff_drive_model.cpp
│   ├── ackermann_model.cpp
│   ├── mecanum_model.cpp
│   └── trailer_model.cpp
│
└── teb_controller_plugin/    ← nav2 controller
```

以后新增车型：

只需要新增一个文件：

```
teb_kinematics/new_vehicle_model.cpp
```

算法主干永远不动。

这就是工业级设计。

------

# 八、为什么这是唯一正确方案

如果不这样做，TEB会变成：

```
if(diff_drive) { ... }
else if(ackermann) { ... }
else if(mecanum) { ... }
else if(trailer) { ... }
```

3年后：

没人敢改。

导航系统死亡。

这在很多公司真实发生过。

------

# 九、一句话总结

当**同一个算法要支持多车型**时：

> 把 “算法” 和 “运动学” 解耦
>  让车型变成 **约束插件**

这就是机器人导航真正的架构设计。





# nav2执行流程

在amr_navigator.hpp中有std::unique_ptr<amr_behavior_tree::BtActionServer<ActionT>> bt_action_server_

该对象在创建时，传进了 std::bind(&Navigator::onGoalReceived,this,std::placeholders::_1),

​            std::bind(&Navigator::onLoop, this),

​            std::bind(&Navigator::onPreempt,this,std::placeholders::_1),

​            std::bind(&Navigator::onCompletion,this,std::placeholders::_1,

这四个模版函数类对象。

BtActionServer类是对SimpleActionServer类的进一步封装，用于驱动bt.但底层还是通过SimpleActionServer类对象中的executeCallback函数来执行的。此外，上面的四个回调也是通过executeCallback来进行调用的。



