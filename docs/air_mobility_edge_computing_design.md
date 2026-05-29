# 面向低空城市管理的移动边缘计算框架：多无人机协同与跨区域任务调度

本方案面向 **低空城市管理（UAV）** 场景，将 **移动边缘计算（MEC）** 引入空域协同：**算力与低时延服务部署在贴近空域的边缘节点上**；无人机在空域中 **连续移动**，其 **所关联的服务边缘（serving edge）随空间位置变化而切换**。这与地面蜂窝中 UE 移动导致 **MEC 关联更新** 是同构问题，强调的是 **移动性下的边缘服务与云边协同**。

> 状态：**仅方案文档**，尚未改代码。  
> 目的：在云边融合框架下描述 **低空城市管理中的 MEC 协同框架**；**`m` 个边缘节点** 与 **`m` 个空域分区** 一一对应，**`m` 为实验参数**（如 2、3）。

---

## 1. 背景与目标（云边融合 + MEC）

- **云**：模型训练、长期优化、全局统计（仿真中可抽象为离线训练与参数下发）。
- **边**：**`m` 个边缘节点（MEC）**，各覆盖一块空域 **Region 0 … Region `m−1`**，向进入该覆盖范围的 UAV 提供 **低时延计算与协同服务**（状态汇聚、任务/空域信息服务、可选异构策略参数等）。
- **端**：**无人机**执行局部感知—决策—执行闭环；强化学习主体默认仍为 **每架 UAV 一个 agent**（MAPPO 多 Actor），边缘为 **服务提供方与环境元数据**，除非单独引入可学习的边缘策略。

### 1.1 移动性与边缘关联

- UAV **可跨区域连续飞行**；任务从 `start` 到 `goal` 的轨迹可穿越多个 Region。
- **任一时间步**，UAV 由 **当前位置所在分区** 对应的边缘节点提供服务关联（实现中可记录 **edge association handover**，与蜂窝/MEC 文献中的 handover 同义）。
- **`m`** 为可配置参数；实验中取 `m=2`、`m=3` 等仅影响分区粒度与关联切换频次，不改变整体 MEC 框架结构。

与地面移动边缘计算的一般建模保持一致，本文将 **association** 明确定义为 `UAV ↔ Edge` 的服务关联关系（由 UAV 所在分区决定）。当 UAV 跨区飞行时，association 发生切换（handover）。严格说不是“任务本体迁移”，而是服务上下文迁移/重绑定：即 UAV 从 Edge-a 切到 Edge-b 后，原来在 Edge-a 维护的该 UAV 服务状态（任务上下文、调度状态、缓存/会话）需要在 Edge-b 可继续服务。在本方案中，任务与无人机之间的 `Task ↔ UAV` 配对属于执行层调度语义，而 `UAV ↔ Edge` 属于服务层语义；边缘侧负责最近派单，端侧负责时效与安全协作执行。

### 1.2 与当前代码的关系（概念上）

- 仿真仍可为 **一个 `MultiUAVEnv`、固定 `n_uavs`、MAPPO 多 Actor + 共享 Critic**。
- 在环境之上增加：**`m` 区域划分**、**区域属性**、**边缘关联切换统计**等；不改变「agent = UAV」的默认设定。

### 1.3 相关研究与应用定位（面向 UAM + MEC）

- 本方案的建模与研究脉络可以归入 **UAV 支持的 Mobile Edge Computing / MEC**，并进一步与 **Urban Air Mobility（UAM）低空机动场景**对齐：无人机作为移动空中终端/服务载体，在飞行过程中触发边缘服务关联更新，从而带来低时延边缘服务与协同执行的需求。
- 典型相关工作方向包括：**多 UAV 辅助 MEC**（如无人机轨迹/部署控制 + 计算资源分配）、以及 **用户移动导致的边缘关联更新（handover）**。例如，参考文献 *Multi-Objective Optimization for Multi-UAV-Assisted Mobile Edge Computing*（Geng Sun 等，IEEE TMC, 2024）研究多 UAV 辅助 MEC 下的任务编排与资源分配，并显式考虑 UAV 轨迹控制与移动性影响；Guo 和 Peng 在 *Efficient mobility management in mobile edge computing networks: Joint handover and service migration*（IEEE Internet of Things Journal, 2023）中进一步将 handover 与 service migration 联合建模，用于提升移动场景下的服务连续性与系统效率。
- 与上述研究相比，本方案的差异与重点可表述为：在“移动性下的边缘服务关联切换”叙事下，将其与**多无人机局部协同控制（MAPPO）**、以及**任务驱动的执行与编排**结合，强调的是“关联切换带来的协同与任务完成性能变化”，而非像UAV-assisted MEC这种仅优化通信/计算卸载目标。

---

## 2. 核心概念与对象

### 2.1 区域（Region）= 边缘覆盖区

- 世界坐标 **互斥且完备** 划分；建议第一版采用 **XY 网格 Region**：
  - 在水平面（XY）将 $[0,W_x]\times[0,W_y]$ 按 $n_x\times n_y$ 网格划分；
  - 垂直方向（Z）不做额外分层：所有 UAV 都归属于同一“单层高度 bin”，因此 Region 总数为 $m=n_x\,n_y$。

  令水平网格索引为 $(i_x,i_y)$，则 Region $k$ 可由组合索引确定，例如：

  $$k=i_x+n_x\,i_y.$$

### 2.2 边缘节点（Edge / MEC）

- **Edge-k** 与 **Region-k** 一一对应，$k \in \{0,\ldots,m-1\}$。
- 职责（现阶段实现）：
  - 本分区内 **服务状态**（队列、活跃 UAV 数等）。
  - **关联切换记录**（不用于目标函数的优化）：UAV 从分区 $i$ 进入分区 $j$ 时记一次 **edge handover**。
  - **关联切换计数**：令 serving edge 为 $e_i(t)=R(\mathbf{p}_i(t))$。则可由相邻时间步是否变化得到
  
  $$
  \text{handover}_i=\sum_{t=1}^{T-1}\mathbb{I}\!\left(e_i(t)\neq e_i(t-1)\right),
  $$
  
  并将 $\text{handover}_i$ 或其累计结果写为 `association_handover_count`（实验评估/日志使用）。
  - **服务容量/负载差异**：每个 Region 设置容量 $C_r$，以当前在该 Region 内的关联 UAV 数量 $N_r(t)$ 近似负载，并在编排/派单阶段用于估计拥塞与时延影响。

### 2.3 无人机（UAV）

- MAPPO **agent**；观测→动作；位置连续演化，**自然穿越分区边界**。

### 2.4 任务（Task）

- `start`、`goal`，及 `priority`、`urgency`、 `deadline`。
- 任务点高度假设：`start.z = 0`、`goal.z = 0`（地面任务点）；UAV 的高度 `z` 不固定，由空间分配与控制策略在执行中决定。
- **跨分区**：`region(start)` 与 `region(goal)` 可不同；轨迹可经过多个 Region。
- 任务池与调度：所有 pending 任务进入同一个全局任务池；边缘侧编排/派单从全局池中选择待分配任务，并激活可用 UAV 执行。
- 任务进度：跨边界 **不重置** 任务进度；任务执行沿着 `start→goal` 连续推进。

---



## 3. 形式化建模与目标函数

为了在论文中清晰地描述“边缘侧任务编排 + UAV 移动导致的服务关联切换”，可将该 MEC 系统形式化为离散时间动态决策问题。在每个时间步 $t$ 同时包含边缘侧的编排/派单决策与端侧的多无人机协同执行。

【体现后面的多个目标】



### 3.1.1 时间、区域与服务关联

- 时间步：$t=0,1,\dots,T-1$。
- 区域划分：$R(\mathbf{x})\in\{0,1,\dots,m-1\}$。
- UAV $i$ 的 serving edge（服务边缘）由其位置决定：$e_i(t)=R(\mathbf{p}_i(t))$。
- 关联切换（edge handover）可用 handover 次数度量：

$$
\text{handover}_i=\sum_{t=1}^{T-1}\mathbb{I}\!\left(e_i(t)\neq e_i(t-1)\right).
$$

### 3.1.2 任务模型

任务集合记为 $\mathcal{K}$。对任意任务 $k\in\mathcal{K}$，给定：

- 起点/终点：$\mathbf{s}_k, \mathbf{g}_k$；
- 属性：priority $p_k$、urgency $u_k$、deadline $d_k$；
- 生命周期：pending / assigned / in\_progress / done / expired。

为保证系统叙事与物理过程一致，本文采用“完整执行链路”：
$$
\mathbf{p}_i(t_{\text{assign}})\rightarrow \mathbf{s}_k \rightarrow \mathbf{g}_k.
$$
即 UAV 被分配任务后，先从当前所在位置飞往任务起点，再进入任务执行段。对应地，任务状态可细分为：

- pending（待分配）；
- in\_progress（任务执行中）；
- done / expired。

### 3.1.3 总体设计

- **边缘侧**：采用可解释的最近距离派单规则；
- **端侧**：采用单阶段 reward 学习协作执行策略。

### 3.1.4 边缘规则：按需任务编排/派单

定义边缘侧策略（第一版为规则模块）：
$$
\pi_{\text{edge}}:\ (\text{task pool},\ \text{UAV status},\ \text{region info})\rightarrow \text{dispatch/activation decisions}.
$$

其角色定位为“轻量调度器”，不承担复杂优化。边缘侧仅执行最近距离派单：

1. 对每个待分配任务 $k$，在可用 UAV 集合 $\mathcal{U}_{\text{avail}}$ 中选择
$$
i^*=\arg\min_{i\in \mathcal{U}_{\text{avail}}}\ \|\mathbf{x}_i-\mathbf{s}_k\|.
$$
2. 将任务 $k$ 分配给 $i^*$，并在任务起点激活执行，UAV 状态置为 `in_progress`。
3. 若存在容量/并发约束，则在约束可行域内按最近距离原则选择。

### 3.1.5 目标函数：任务效率+空间+拥堵+切换

端侧对每架 UAV 学习分布式策略 $\pi_i(a_i(t)\mid o_i(t))$，集中式 critic 使用全局观测（CTDE）。

本方案主要创新点定位在端侧协作执行：在动态任务与跨区域移动场景下，学习多 UAV 的协同避碰、时效执行与空间资源协调能力。边缘层仅负责按需分配，不参与端侧策略参数更新。

在“接单后直接于任务起点执行（视作瞬移）”设定下，端侧将单阶段 reward 分解为主目标与增强项：

主目标（任务效率+空间+拥堵+切换）：
$$
G_i(t)=
w_{\text{task}}\cdot\Big(p_i u_i\cdot \Delta y_i(t)\Big)
+w_{\text{safe-corr}}\cdot J_{i,t}^{\text{safe-corr}}
+w_{\text{cong}}\cdot\text{CongestionMetric}(t)
-w_{\text{switch}}\cdot J_{i,t}^{\text{switch}}.
$$

增强项（一次性完成奖励+靠近终点方向增强）：
$$
E_i(t)=
w_{\text{comp}}\cdot\Big(p_i u_i\cdot \mathbb{I}[\text{completed\_now}_i]\Big)
+w_{\text{dist}}\cdot\left(d_i(t-1)-d_i(t)\right).
$$

因此单阶段 reward 为：
$$
r_i(t)=G_i(t)+E_i(t).
$$

为与实现保持一致，本文将原“安全惩罚（safety penalty）”统一改称为**安全空间修正（safety space correction）**，定义为：
$$
J_{i,t}^{\text{safe-corr}}=-\left|\tilde{v}_i^t-v_i^t\right|,
$$
其中 $v_i^t$ 为 UAV 在时刻 $t$ 提议的空间尺度动作，$\tilde{v}_i^t$ 为经安全投影层修正后的可执行动作。该项刻画“动作被安全层修正的幅度”，修正越大，回报越低。

该写法将学习信号集中在执行阶段：任务推进、完成奖励、安全空间修正与拥塞约束共同驱动协作执行质量。

其中 `J_{i,t}^{switch}` 表示分区切换的“超额成本”（budget 超额惩罚）：
$$
J_{i,t}^{\text{switch}}
=\mathbb{I}\!\left(r_i(t)\neq r_i(t-1)\right)
+\kappa\cdot \max\!\left(0,\ \text{gridDist}\!\left(r_i(t-1),r_i(t)\right)-1\right),
$$
其中：
$$
r_i(t)=\text{region}\!\left(\mathbf{p}_i(t)\right)
$$
为 UAV `i` 在时刻 `t` 的分区索引；`\text{gridDist}` 为两分区间的网格曼哈顿距离（若水平网格索引为 $(i_x,i_y)$，则
$\text{gridDist}(k_1,k_2)=|i_{x,1}-i_{x,2}|+|i_{y,1}-i_{y,2}|$）。
第一项表示只要切区就惩罚 1；第二项对“跳区”（距离大于 1 的非相邻切换）额外惩罚，并由 $\kappa$ 控制强度。

为避免惩罚过强导致 UAV 在当前区域徘徊，本文采用“预算超额”版本的切换成本（覆盖上述逐步惩罚表述）：
$$
J_{i,t}^{\text{switch}}
=\max\!\left(0,\ N_i^{\text{switch}}(t)-B_i\right),
$$
其中 $r_i^{\text{start}}$ 与 $r_i^{\text{goal}}$ 分别为当前任务起点/终点分区，
$$
h_i=\text{gridDist}\!\left(r_i^{\text{start}},r_i^{\text{goal}}\right),\qquad
B_i=h_i+\delta\ (\delta\ge 0),
$$
并从任务开始时刻 $t_0$ 开始统计累计切换次数：
$$
N_i^{\text{switch}}(t)=\sum_{\tau=t_0+1}^{t}\mathbb{I}\!\left(r_i(\tau)\neq r_i(\tau-1)\right).
$$

并定义距离增强项所用的分区距离：
$$
d_i(t)=\text{gridDist}\!\left(r_i(t),r_i^{\text{goal}}\right).
$$

训练与评估指标在本节统一解释（episode/window 统计）：

- 完成率、按时完成率、平均完成时延；
- 安全相关指标（冲突/近失率等）；
- 空间拥塞指标采用区域利用率方差：
$$
\rho_r(t)=\frac{\sum_{i\in\mathcal{N}_r(t)}\tilde{v}_i^t}{V_r},\qquad
\bar{\rho}(t)=\frac{1}{|\mathcal{R}|}\sum_{r\in\mathcal{R}}\rho_r(t),
$$
$$
\text{CongestionMetric}(t)=\frac{1}{|\mathcal{R}|}\sum_{r\in\mathcal{R}}\left(\rho_r(t)-\bar{\rho}(t)\right)^2.
$$
其中 $\tilde{v}_i^t$ 为 UAV $i$ 在时刻 $t$ 经安全投影后的可执行空间动作，$\mathcal{N}_r(t)$ 为时刻 $t$ 位于区域 $r$ 的 UAV 集合，$V_r$ 为区域 $r$ 的总可用空间体积。该指标越大表示区域利用越不均衡、拥塞风险越高。

### 3.1.6 MAPPO 协作执行：实现目标函数

为最大化 3.1.5 中定义的端侧目标函数（主目标 + 增强项），本文采用 MAPPO 的协作训练框架，并使用 CTDE（centralized training, decentralized execution）实现分工优化：

1. **Actor（每 UAV 一套）**：学习分布式策略 $\pi_i(a_i(t)\mid o_i(t))$，在执行阶段仅使用本机观测 $o_i(t)$ 输出动作。
2. **Critic（集中式）**：使用共享 critic 在全局观测（或全局状态的拼接）下估计价值 $V_\phi$，用于计算优势并稳定训练。
3. **目标与 reward**：每步 reward 按 $r_i(t)=G_i(t)+E_i(t)$ 构造，其中
   - $G_i(t)$（任务效率+空间+拥堵+切换）为主体优化项；
   - $E_i(t)$（一次性完成奖励、以及靠近终点的方向增强）作为 reward shaping 辅助引导策略。


---

## 4. MAPPO 与多边缘 MEC

- **训练**：CTDE — 多 UAV Actor + 共享 Critic；或后续分域 Critic。
- **执行**：每机只跑自身 Actor。
- **边缘不增加默认 MAPPO agent 数量**：**agent 数 = UAV 数**；多边缘体现为 **空间分区上的 MEC 关联与服务参数**。

「策略上分区域」为显式扩展，见第 8 节。

---

## 5. 数据集模拟

### 5.0 任务位置与到达生成

- 任务到达数量/时机可按泊松过程生成（用于建模动态任务流）。
- 任务位置在 XY 平面采样（可均匀/热点分布）；任务点高度固定为地面层：`start.z = 0`、`goal.z = 0`。

### 5.1 deadline 模拟（必选属性）

- 任务到达时记录 `release_timestamp_k`，并设置绝对截止时刻 `deadline_timestamp_k`。
- 每个仿真步对未完成任务计算 `deadline_remaining_steps_k = deadline_timestamp_k - current_timestamp`；当其小于 0 且任务仍未完成时，任务状态置为 `expired`。
- 使用绝对截止步可自然覆盖 `pending`、`assigned`、`in_progress` 各阶段的时间消耗，避免仅用“执行剩余步数”带来的偏差。

### 5.2 deadline 采样与可行性

- deadline 由任务几何长度与单位速度换算得到基准执行时间，再乘以 slack 系数 $\rho$：

$$
T_k^{\text{base}}=\frac{\|\mathbf{g}_k-\mathbf{s}_k\|}{v_0},\qquad
deadline\_timestamp_k=release\_timestamp_k+\left\lceil \rho \, T_k^{\text{base}}\right\rceil.
$$

其中 $v_0=1$（单位速度）可作为统一基准，$\rho>1$ 为 slack 系数（越小越紧，越大越松）。该设置可直接用于实验对比：固定 $\rho$ 或在若干 $\rho$ 档位下做消融。

### 5.3 与派单与评估的对齐

- 第 `3.1.4` 节采用“最近 UAV 接单”规则，不再构造边缘代价函数。
- deadline 影响主要由端侧执行速度与任务完成阶段奖励/惩罚体现，并通过 `on-time completion rate` 与 `mean tardiness` 评估。

---

## 6. 系统行为：跨分区与边缘关联切换

### 6.1 难点分析：移动性下边缘任务编排/调度的挑战

在该低空城市管理 MEC 设定下，“边缘侧任务编排/派单”面对的关键难点并不在于任务本身的几何起终点，而在于**执行过程中的边缘关联随移动不断切换**，从而引入多源不确定性与非平稳性。主要体现在：

1. **边缘关联切换顺序不可由 start/goal 直接推导**
   - 虽然每架 UAV 的任务起点 `start` 与终点 `goal` 在 spawn 后固定，但 UAV 的实际轨迹并非由几何单调决定。
   - 轨迹会受到（i）导航与位移缩放、（ii）安全投影/冲突消解、（iii）多无人机相互推挤的耦合影响，因此跨 Region 的边界穿越时刻与先后顺序会随策略动作变化而改变。
   - 结论：边缘节点无法在离线仅凭 start/goal 预先得到“必然的服务关联切换序列”，只能面向在线观测与统计做编排决策。

2. **服务语义变化导致“调度决策—执行过程”耦合**
   - UAV 所关联的 serving edge（对应 Region）变化后，边缘侧的服务可用性与容量/负载特征会随之改变（例如拥塞水平与可服务并发数）。
   - 因此编排/派单决策不仅要选择“派给谁”，还要保证该决策与后续服务语境变化相兼容，否则会出现等待、失败或安全约束频繁触发。

3. **非平稳性与鲁棒性需求**
   - 对任一边缘节点而言，进入/离开其覆盖范围的 UAV 集合是动态的；同时 UAV 的相对位置与任务进度也在变化。
   - 这使得从边缘视角看，任务到达与可执行能力都呈现非平稳特征，要求编排策略具备鲁棒性（例如减少过度重分配、在关联抖动下保持稳定）。

4. **多无人机耦合放大调度不确定性**
   - 多 UAV 同时移动、共享空间约束与安全投影会引入强耦合：一个 UAV 的避碰修正会改变另一个 UAV 的位置演化，进而改变服务关联切换与奖励反馈。
   - 这使得边缘编排不是“单任务独立优化”，而是要在群体协同下形成长期有效的派单与激活逻辑。

5. **实验指标需要同时覆盖“任务完成”与“关联切换”**
   - 仅看任务完成率/完成时间可能掩盖策略在边缘关联切换方面的质量。
   - 建议联合统计：完成率、完成步数/完成时延、跨分区任务比例、以及 serving edge association handover 次数与频率，用于解释“为何编排策略在某些场景更有效”。

## 7. 实验设计

### 7.1 指标设计

- 协作执行主指标：完成率、平均完成步数/时间、`on-time completion rate`、`expiration rate`、`mean tardiness`。
- 协作安全与效率：训练回报曲线。
- 边缘侧支撑指标：跨分区任务比例；单轨迹穿越分区数；**边缘关联切换次数**。
- 负载状态指标：区域利用率方差 $\mathrm{Var}(\rho_r)$、超载时长（或超载步数）与区域利用率分布（均值/分位数）。

### 7.2 简化策略消融（主力验证）

- 目的：验证“最近派单 + 端侧单阶段 reward”是否能稳定提升时效与完成质量。
- 边缘侧消融：最近派单 vs 随机派单 vs 轮询派单。
- 端侧消融：分别去掉 reward 中对应协作项（如 `completion` / `safe-corr` / `congestion` / `switch` / `dist-shaping`），观察协作执行质量变化。
- 对比方式：固定边缘规则看端侧项作用；固定端侧奖励看派单规则作用。
- 预期现象：最近派单应降低平均完成时延；端侧完整奖励应在 `on-time` 与安全指标上最优。

### 7.3 基础对比设置

- 消融：不同 **`m`**（如 2 vs 3）。

## 8. 策略上分区域（后续）

- 参数分域 / MoE；Critic 先共享。

---

## 9. 讨论点

- 切换成本的权重要调小一点，不然担心在任务

---

## 10. 配置项草案

```yaml
regions:
  enabled: true
  count: m
  # Region 由 XY 网格 (n_x, n_y) 构成，Z 不做分层（单层 bin），m = n_x*n_y
  region_params:
    - { capacity: 1.0 }
    # ... 长度 m
```

---

## 11. 待讨论清单

