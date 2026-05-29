# Task-Oriented Multi-UAVs

面向任务驱动的多无人机协同空域仿真项目。采用 **MAPPO**（Multi-Agent Proximal Policy Optimization）进行集中式训练、分布式执行的多智能体强化学习，支持动态任务调度、可插拔导航与安全投影层。

## 快速开始

### 依赖

```bash
pip install numpy torch pyyaml matplotlib
```

### 启动训练

项目根目录下执行：

```bash
python train.py
```

默认读取 `configs/demo_config.yaml`。也可指定配置文件：

```bash
python train.py --config configs/demo_config.yaml
```

### 仅从日志绘制收敛曲线

不重新训练，根据已有 reward 日志生成曲线图：

```bash
python train.py --plot-from-log checkpoints/reward_log.txt
```

## MAPPO 启动入口

| 层级 | 文件 | 说明 |
|------|------|------|
| **训练入口** | [`train.py`](train.py) | 命令行入口，`if __name__ == "__main__"` 解析参数后调用 `train()` |
| **算法封装** | [`algorithms/mappo.py`](algorithms/mappo.py) | `MAPPO` 类与 `MAPPOConfig`；每架 UAV 一个 Actor，共享一个 Critic |
| **网络结构** | [`algorithms/networks.py`](algorithms/networks.py) | `ActorNetwork`、`CriticNetwork` |
| **环境** | [`environments/multi_uav_env.py`](environments/multi_uav_env.py) | `MultiUAVEnv` 仿真环境 |
| **配置** | [`configs/demo_config.yaml`](configs/demo_config.yaml) | 环境、算法超参、训练与调度参数 |

训练主流程（`train.py` → `train()`）：

1. `load_config()` 加载 YAML 配置
2. `make_env()` 构建 `MultiUAVEnv`（含导航器、任务调度器）
3. `make_mappo()` 实例化 `MAPPO` 算法
4. 循环采样 episode → 写入 buffer → 按 `update_freq` 调用 `algo.update_policy()`
5. 按 `save_freq` 导出轨迹 JSON，训练结束后保存 reward 数据与曲线图

## 训练输出

输出目录由配置项 `training.save_dir` 控制，默认为 `checkpoints/`。

| 输出文件 | 说明 |
|----------|------|
| `checkpoints/reward_log.txt` | 每行 `episode,mean_reward`，训练过程中实时写入 |
| `checkpoints/reward_history.npy` | 全部 episode 平均 reward 的 NumPy 数组，训练结束时保存 |
| `checkpoints/reward_curve.png` | 收敛曲线图（原始 reward + 滑动平均） |
| `checkpoints/json/mappo_ep{N}.json` | 第 N 个 episode 的轨迹导出（`save_freq` 间隔生成），格式为 `{"devices": [...]}`，供前端可视化，把json文件拖到https://pharos-3d.ezzo.cc/中看效果 |

轨迹 JSON 中每条记录包含无人机 `uid`、位置、速度、时间戳、包围盒 `include_area`、目标点 `target_pos`、任务得分 `task_score` 等字段。

> 当前实现按 `save_freq` 仅导出轨迹 JSON，**不保存** `.pt` 模型权重文件。

## 代码结构

```
task-oriented-UAVs/
├── train.py                      # MAPPO 训练启动入口
├── configs/
│   └── demo_config.yaml          # 默认训练配置
├── algorithms/
│   ├── __init__.py
│   ├── mappo.py                  # MAPPO 算法（RolloutBuffer、策略更新）
│   └── networks.py               # Actor / Critic 神经网络
├── environments/
│   ├── __init__.py
│   └── multi_uav_env.py          # 多无人机仿真环境（观测、奖励、step）
├── tasks/
│   ├── task.py                   # Task 数据结构与 TaskGenerator
│   └── scheduler.py              # 贪心任务分配调度器
├── navigation/
│   ├── __init__.py               # get_navigator() 工厂函数
│   ├── base.py                   # BaseNavigator 抽象基类
│   └── astar.py                  # A* 路径导航实现
├── safety/
│   ├── safe_layer.py             # SafeProjectionLayer 安全动作投影
│   └── collision_checker.py      # 立方体冲突检测
├── checkpoints/                  # 训练输出（日志、曲线、轨迹 JSON）
├── docs/                         # 系统设计文档
└── related_paper/                # 相关参考文献
```

### 模块职责

- **algorithms**：MAPPO 多智能体 PPO 实现，CTDE 架构（各 Actor 独立，Critic 使用全局观测）
- **environments**：空域仿真；支持传统固定起终点模式与「任务池 + 规则调度」动态任务模式
- **tasks**：动态任务生成与 `greedy_assign_pending_tasks` 规则分配
- **navigation**：可插拔导航（`astar` / `none`），与 RL 策略解耦
- **safety**：将不安全动作投影到满足间距与尺寸约束的安全动作

## 主要配置项

`configs/demo_config.yaml` 中的关键参数：

```yaml
training:
  n_episodes: 500      # 训练 episode 数
  max_steps: 100       # 每个 episode 最大步数
  update_freq: 50      # 每 N 个 episode 更新一次策略
  save_freq: 50        # 每 N 个 episode 导出轨迹 JSON
  save_dir: "checkpoints"

algorithm:
  actor_lr: 0.0003
  critic_lr: 0.001
  gamma: 0.99
  ppo_epochs: 10

task_scheduler:
  enabled: true        # 启用动态任务调度
```

更多系统设计说明见 [`docs/air_mobility_edge_computing_design.md`](docs/air_mobility_edge_computing_design.md)。
