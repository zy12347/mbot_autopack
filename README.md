# mbot_autopack 算法模块说明

## Navigation
### RRT (快速探索随机树)

#### 算法简介
RRT（Rapidly-exploring Random Tree）是一种一种基于采样的路径规划算法，通过在自由空间中随机采样并逐步扩展树状结构来寻找从起点到目标点的路径。适用于高维空间和复杂环境的路径规划问题。

#### 核心实现
- 位于 `src/navigation/src/RandomTree.cpp` 和 `src/navigation/include/RandomTree.h`
- 主要功能：
  - `PlanPath()`: 核心路径规划函数，通过迭代扩展随机树寻找路径
  - `BuildPath()`: 根据找到的目标节点回溯构建完整路径
  - `IsPathPassable()`: 使用Bresenham算法检查路径是否可通行
  - `PlotMap()`/`PlotTree()`: 可视化路径和随机树结构

#### 使用方法
```cpp
// 初始化RRT规划器
RandomTree rrt(map_width, map_height, map_data);
// 规划路径（起点、终点、步长、最大迭代次数）
std::vector<std::pair<int, int>> path = 
    rrt.PlanPath(start_x, start_y, target_x, target_y, step_size, max_iterations);
// 可视化结果
rrt.PlotTree();
rrt.PlotMap(path);
```

#### 参数说明
- `step_size`: 每次扩展的步长，影响路径平滑度和探索效率
- `max_iterations`: 最大迭代次数，平衡计算时间和路径发现成功率


### A* (A星算法)

#### 算法简介
A* 是一种基于启发式搜索的路径规划算法，结合了Dijkstra算法的完整性和贪婪最佳优先搜索的效率，通过评估函数指引搜索方向，能找到最优路径。

#### 核心实现
- 位于 `src/navigation/src/Astar.cpp` 和 `src/navigation/include/Astar.h`
- 主要功能：
  - `PlanPath()`: 核心路径规划函数
  - `GetMinScoreIndex()`: 寻找Open列表中代价最小的节点
  - `HeuristicScore()`: 计算启发式代价（使用曼哈顿距离）
  - `ProcessNeighbor()`: 处理当前节点的邻居节点
  - `BuildPath()`: 从目标节点回溯构建路径
  - `PlotMap()`: 可视化路径

#### 算法原理
- 评估函数：`f(n) = g(n) + h(n)`，其中：
  - `g(n)`: 从起点到当前节点的实际代价
  - `h(n)`: 从当前节点到目标节点的估计代价（启发式函数）
- 采用曼哈顿距离作为启发式函数，保证算法的最优性

#### 使用方法
```cpp
// 初始化A*规划器
Astar astar(map_width, map_height, map_data);
// 规划路径（起点和终点坐标）
std::vector<std::pair<int, int>> path = 
    astar.PlanPath(start_x, start_y, target_x, target_y);
// 可视化路径
astar.PlotMap(path);
```
## SLAM
### GMapping (基于粒子滤波的SLAM)

#### 算法简介
GMapping是一种基于粒子滤波的激光SLAM（同步定位与地图构建）算法，能够在未知环境中同时估计机器人位姿并构建环境地图。

#### 核心实现
- 位于 `src/slam/src/gmapping/gmapping.cpp` 和 `src/slam/src/gmapping/gmapping.h`
- 主要功能：
  - `Initialize()`: 初始化粒子群
  - `ProcessScan()`: 处理激光扫描数据的主流程
  - `Predict()`: 根据里程数据预测粒子位姿
  - `OptimizePose()`: 优化粒子位姿
  - `ComputeAndNormalizeWeights()`: 计算并归一化粒子权重
  - `Resample()`: 根据权重重采样粒子
  - `UpdateMap()`: 更新地图

#### 工作流程
1. 初始化：设置初始位姿和粒子群
2. 预测：根据里程计数据预测每个粒子的新位姿
3. 优化：使用扫描匹配优化粒子位姿
4. 权重计算：根据激光数据与地图匹配度计算粒子权重
5. 重采样：根据权重选择粒子，保持粒子多样性
6. 地图更新：使用最优粒子的位姿更新全局地图

#### 参数说明
- `particle_count`: 粒子数量，影响定位精度和计算量
- `map_resolution`: 地图分辨率，单位：米/栅格
- `nerf_threshold`: 重采样阈值，控制重采样频率

## 模块间关系
- GMapping模块生成的地图数据（`GridMap`）可作为A*和RRT算法的输入
- 导航节点（`navigation_node.cpp`）提供了算法测试示例，可加载地图并可视化路径规划结果
- 所有算法均使用栅格地图表示环境，通过`GridMap`类进行坐标转换和地图操作