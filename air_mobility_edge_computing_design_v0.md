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

与地面移动边缘计算的一般建模保持一致，本文将 **association** 明确定义为 `UAV ↔ Edge` 的服务关联关系（由 UAV 所在分区决定）。当 UAV 跨区飞行时，association 发生切换（handover）。严格说不是“任务本体迁移”，而是服务上下文迁移/重绑定：即 UAV 从 Edge-a 切到 Edge-b 后，原来在 Edge-a 维护的该 UAV 服务状态（任务上下文、调度状态、缓存/会话）需要在 Edge-b 可继续服务。在本方案中，任务与无人机之间的 `Task ↔ UAV` 配对属于执行层调度语义，而 `UAV ↔ Edge` 属于服务层语义；两层通过 Delay 与 CongestionCost（平均负载）在系统目标中耦合。

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
- assigned\_to\_pickup（已分配，飞往起点）；
- in\_progress（从起点到终点执行中）；
- done / expired。

### 3.1.3 系统总目标（多目标）

为统一“边缘按需编排 + 端侧协作执行”两层行为，可将系统目标表述为：
$$
\max\ \mathbb{E}\Big[\sum_{t=0}^{T-1}\gamma^t\big(
\text{CompletionUtility}(t)-w_{\text{delay}}\,\text{Delay}(t)-w_{\text{cong}}\,\text{CongestionCost}(t)
\big)\Big]
\quad \text{s.t. Safety}.
$$

其中，`CompletionUtility` 体现任务完成质量，`Delay` 体现时效，`CongestionCost` 体现系统拥塞成本，`Safety` 为安全约束。后续 `3.1.4` 与 `3.1.5` 分别给出边缘与端侧对该目标的近似优化方式。

在 `3.1.3` 中，拥塞项采用区域负载率的平均值：
$$
\text{CongestionCost}(t)=\frac{1}{|\mathcal{R}|}\sum_{r\in\mathcal{R}}\frac{N_r(t)}{C_r},
$$
其中 $N_r(t)$ 为时刻 $t$ 区域 $r$ 的在服 UAV 数量，$C_r$ 为区域容量。

为避免“任务质量”在两层出现重复定义，本文统一采用单一定义：
$$
\text{CompletionUtility}(k)=(p_k\cdot u_k)\cdot \mathbb{I}[\text{on\_time\_done}_k],
$$
其中 $\mathbb{I}[\text{on\_time\_done}_k]$ 表示“任务 $k$ 在 deadline 前完成”指示函数。等价地，系统层可写为加权按时完成数量：
$$
\sum_k (p_k\cdot u_k)\cdot \mathbb{I}[\text{on\_time\_done}_k].
$$

系统项到两层代理项的映射关系可总结为：

| 系统目标项（3.1.3） | 边缘规则代理（3.1.4） | 端侧学习代理（3.1.5） |
|---|---|---|
| $\text{CompletionUtility}$ | 通过 $(p_k\!\cdot\!u_k)$ 与 $\widehat{\text{lateness\_risk}}$ 近似其“价值+时限”语义 | 通过 $w_{\text{task}}\cdot\Delta y + w_{\text{comp}}\cdot\mathbb{I}[\text{completed}]$ 近似其“推进+完成”语义 |
| $\text{Delay}$ | $\widehat{\text{Delay}}$, $\widehat{\text{lateness\_risk}}$ | pickup/execution 阶段效率项（含步数/效率惩罚） |
| $\text{CongestionCost}$ | $\widehat{\text{Congestion}}$ | 通过空间均衡/局部拥塞惩罚间接对齐 |
| $\text{Safety}$（约束） | 规则层不直接学习，主要通过代价约束间接体现 | $w_{\text{safety}}$ 惩罚项（显式） |

按论文写作逻辑，可将本节的建模路径概括为三步：

1. 先定义全局目标函数（完成质量/时延/拥塞成本/安全约束）；
2. 再说明该目标难以直接优化：包含离散分配与连续控制耦合、组合空间爆炸、非凸性、以及动态任务与多机耦合带来的时变非平稳性；
3. 最后提出分解近似：上层采用启发式/规则编排，下层采用 RL 协作学习，并通过目标一致性假设给出 surrogate 层面的理论说明。

因此本文不主张“公式 10/11 与公式 4 严格等价”，而将公式 10/11 视为公式 4 的 tractable surrogates（可计算代理目标）。

可进一步将该问题定义为 bilevel 结构：

- 上层：边缘节点选择任务分配 $a$，最小化编排代价 $C(a)$；
- 下层：给定上层分配 $a$，端侧 MAPPO 最大化协作回报 $J_{\pi}(a)$。

并给出“目标一致性”假设：

- 边缘代价 $C(a)$ 中的 Delay/Congestion/Lateness 项与系统总目标中的对应项同向；
- 端侧 reward 中的 progress/completion/safety 项与系统总目标中的完成质量与安全约束同向。

据此，本文将两层优化解释为：在不同可行域上的同一系统目标 surrogate optimization（而非彼此独立的异构目标优化）。

方法论上，本文采用“规则编排 + 协作学习”的分层混合决策框架（hierarchical hybrid framework），边缘层提供可解释且样本高效的任务分配先验，端侧通过 MAPPO 学习高维连续动作下的多机协作执行。该分层设计用于应对动态任务与跨区域移动场景中的非平稳性，并在统一系统目标下实现“可解释决策 + 可学习协作”的协同优化。

### 3.1.4 边缘规则：按需任务编排/派单

定义边缘侧策略（第一版为规则模块）：
$$
\pi_{\text{edge}}:\ (\text{task pool},\ \text{UAV status},\ \text{region info})\rightarrow \text{dispatch/activation decisions}.
$$

其角色定位为“按需编排与分配”的支撑层，而非主要学习创新点。边缘规则通过轻量可解释代价近似系统目标中的时效/拥塞项：
$$
\text{cost}(i,k)=\;w_{\text{cong}}\,\widehat{\text{Congestion}}(i,k)\;+w_{\text{delay}}\,\widehat{\text{Delay}}(i,k)\;+\;w_{\text{late}}\,\widehat{\text{lateness\_risk}}(i,k).
$$

其中：

- $\widehat{\text{Congestion}}(i,k)$：区域拥塞/负载惩罚估计；
- $\widehat{\text{Delay}}(i,k)$：完整链路预计完成步数估计；
- $\widehat{\text{lateness\_risk}}(i,k)$：与任务价值耦合的 deadline 风险项（用于提升按时完成概率，对应 $\text{CompletionUtility}$ 的 on-time 语义）。

建议定义：
$$
\widehat{\text{lateness\_risk}}(i,k)=\widehat{U}_{\text{task}}(k)\cdot\max\!\left(0,\ \frac{\widehat{\text{Delay}}(i,k)-ddl\_remaining\_time_k}{\max(1,ddl\_remaining\_time_k)}\right),
$$
$$
\widehat{\text{Delay}}(i,k)=\left(\frac{\|\mathbf{x}_i-\mathbf{s}_k\|+\|\mathbf{s}_k-\mathbf{g}_k\|}{v_0}\right)\cdot\left(1+\alpha\frac{N_r}{C_r}\right),
$$
$$
\widehat{U}_{\text{task}}(k)=p_k\cdot u_k,
$$
用于对应 $\text{CompletionUtility}(k)$ 中的任务价值项。该时延定义已包含“从 UAV 当前位置到任务起点”的调度移动成本，因此无需额外单列召唤成本项。

然后边缘侧用 greedy 选择满足约束的 dispatch/activation 决策：
$$
\min_{\text{assignments}}\ \sum_{(i,k)\in \mathcal{A}}\text{cost}(i,k),
$$
其中 $\mathcal{A}$ 为被选中的 UAV-任务配对集合。

### 3.1.5 无人机分布式策略学习（MAPPO 协作执行）

端侧对每架 UAV 学习分布式策略 $\pi_i(a_i(t)\mid o_i(t))$，集中式 critic 使用全局观测（CTDE）。

本方案主要创新点定位在端侧协作执行：在动态任务与跨区域移动场景下，学习多 UAV 的协同避碰、时效执行与空间资源协调能力。边缘层仅负责按需分配，不参与端侧策略参数更新。

为避免“飞往任务起点阶段缺乏有效学习信号”，建议按任务状态采用分阶段回报：
$$
r_i(t)=
\begin{cases}
w_{\text{pick}}\cdot \Delta d_i^{\text{pickup}}(t)
+w_{\text{safety}}\cdot\text{(safety penalty)}
+w_{\text{eff}}\cdot\text{(efficiency penalty)}, & z_i(t)=\text{assigned\_to\_pickup},\\[4pt]
w_{\text{task}}\cdot\Big(p_i u_i\cdot \Delta y_i(t)\Big)
+w_{\text{comp}}\cdot\Big(p_i u_i\cdot \mathbb{I}[\text{completed\_now}_i]\Big)
+w_{\text{safety}}\cdot\text{(safety penalty)}
+w_{\text{eff}}\cdot\text{(efficiency penalty)}, & z_i(t)=\text{in\_progress},\\[4pt]
0\ \text{(or idle penalty)}, & z_i(t)=\text{inactive}.
\end{cases}
$$

该写法将同一任务拆分为 pickup/execution 两个 phase，并提供匹配学习信号，使“协作执行质量”成为训练主目标，“编排分配逻辑”作为外部条件输入。其中奖励中的 progress/completion 项用于近似统一定义的 $\text{CompletionUtility}(k)$。

### 3.1.6 理论说明：系统目标与两层代理目标一致性

严格意义上，边缘规则代价与端侧回报并非对 `3.1.3` 系统目标的逐项等价展开，而是其可计算代理（surrogate）：

- 边缘层代理：以 $\text{cost}(i,k)$ 近似系统目标中的时延/拥塞与时限风险；
- 端侧代理：以分阶段 reward 近似系统目标中的完成质量与安全效率项。

可写为分层近似优化：
$$
J_{\text{sys}}\;\approx\;J_{\text{edge-sur}}+J_{\text{uav-sur}},
$$
并在假设估计误差有界时给出一致性说明：
$$
\left|J_{\text{sys}}-\left(J_{\text{edge-sur}}+J_{\text{uav-sur}}\right)\right|\le \epsilon_{\text{edge}}+\epsilon_{\text{uav}},
$$
其中 $\epsilon_{\text{edge}}$ 来自 $\widehat{\text{Delay}}/\widehat{\text{Congestion}}$ 等启发式估计误差，$\epsilon_{\text{uav}}$ 来自 reward shaping 对系统效用的近似误差。该写法用于说明“两层目标方向一致”，而非声称数学上的严格同构。

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

### 5.3 与派单代价和评估的对齐

- deadline 风险项已并入第 `3.1.4` 节的边缘派单代价函数（`cost(i,k)`），用于驱动派单优先关注临近超时任务。

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
- 协作安全与效率：训练回报曲线、冲突/近失事件率（如有）、无效空转步数占比（如有）。
- 边缘侧支撑指标：跨分区任务比例；单轨迹穿越分区数；**边缘关联切换次数**。
- 负载状态指标：区域平均负载比 $N_r/C_r$、超载时长（或超载步数）与负载方差。

### 7.2 surrogate 一致性消融（主力验证）

- 目的：验证公式 10/11 作为公式 4 的 tractable surrogates 是否在方向上对齐系统目标。
- 边缘侧消融：分别去掉 edge cost 中的关键项（`congestion` / `lateness`），观察系统目标相关指标变化。
- 端侧消融：分别去掉 reward 中对应协作项（如 `completion` / `safety` / `efficiency`），观察协作执行质量变化。
- 对比方式：单项去除、仅边缘启用、仅端侧启用、两层同时启用。
- 预期现象：若 surrogate 设计有效，则 `on-time` / 完成率 / 时延 / 拥塞峰值 / 空间失衡度等指标会按项作用方向变化，且“两层同时启用”表现最优。

### 7.3 基础对比设置

- 消融：不同 **`m`**（如 2 vs 3）。

## 8. 策略上分区域（后续）

- 参数分域 / MoE；Critic 先共享。

---

## 9. 观测（讨论点）

- **`region_id`**：one-hot 长度 **`m`**。

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



