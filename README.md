# HybridHydrodynamics

`HybridHydrodynamics` 是一个独立的 Gazebo Sim 系统插件，面向水下航行器以及跨介质飞行器（空气/水过渡载具）使用。它延续了官方 Gazebo hydrodynamics 插件的速度读取方式与 `PreUpdate` 生命周期，并在此基础上加入了面向过渡场景的流体动力学模型，包括阻尼、插件侧对角附加质量、简化的附加质量科里奥利项，以及过渡状态诊断信息。

当前仓库包含一个插件实现：

- 插件别名：`zyshine3_gz_plugins::HybridHydrodynamics`
- 动态库目标：`libhydrodynamics.so`
- 源文件：`hydrodynamics.cpp`、`hydrodynamics.hpp`

## 插件功能概览

在每一个仿真步中，插件会依次执行以下流程：

1. 读取目标 link 的位姿、世界系线速度和世界系角速度
2. 从速度中减去世界坐标系下恒定流场 `default_current`
3. 将相对运动量变换到机体系
4. 根据 `sample_point` 估算艇体浸没程度
5. 计算连续介质缩放因子 `mediumScale`
6. 组装阻尼项
7. 在当前配置和所有权判定允许的情况下，可选加入插件侧附加质量惯性项与简化附加质量科里奥利项
8. 将最终的力/力矩从机体系旋回世界系，并通过 `AddWorldWrench` 施加到模型上

插件还会发布以下过渡诊断主题：

- `/model/<model_name>/transition_state`

## 仓库结构

```text
.
├── CMakeLists.txt
├── hydrodynamics.cpp
├── hydrodynamics.hpp
└── README.md
```

## 依赖项

当前 `CMakeLists.txt` 面向 Gazebo Sim 8，依赖如下：

- `gz-plugin2`
- `gz-sim8`
- `gz-transport13`
- `gz-math7`
- `gz-common5`
- `gz-msgs10`
- `Protobuf`

插件使用 C++17 构建。

## 构建方式

在本地构建：

```bash
cmake -S . -B build
cmake --build build -j
```

生成的共享库路径为：

```bash
build/libhydrodynamics.so
```

执行安装后，目标文件会被放到：

```bash
lib/zyshine3_gz_plugins/
```

## 加载插件

可将其作为 Gazebo Sim 系统插件挂载到模型上：

```xml
<plugin filename="/absolute/path/to/build/libhydrodynamics.so"
        name="zyshine3_gz_plugins::HybridHydrodynamics">
  <hydrodynamics>
    <link_name>base_link</link_name>
    <surface_z>0.0</surface_z>
    <transition_band>0.1</transition_band>
    <fluid_density_water>1000.0</fluid_density_water>
    <fluid_density_air>1.225</fluid_density_air>
    <default_current>0 0 0</default_current>

    <enable_added_mass>true</enable_added_mass>
    <enable_added_mass_coriolis>true</enable_added_mass_coriolis>

    <xDotU>-2.4</xDotU>
    <yDotV>-5.2</yDotV>
    <zDotW>-2.1</zDotW>
    <kDotP>-0.08</kDotP>
    <mDotQ>-0.24</mDotQ>
    <nDotR>-0.32</nDotR>

    <added_mass_velocity_filter_tau>0.03</added_mass_velocity_filter_tau>
    <added_mass_accel_filter_tau>0.06</added_mass_accel_filter_tau>
    <added_mass_linear_accel_limit>4.0</added_mass_linear_accel_limit>
    <added_mass_angular_accel_limit>8.0</added_mass_angular_accel_limit>
    <added_mass_linear_jerk_limit>20.0</added_mass_linear_jerk_limit>
    <added_mass_angular_jerk_limit>40.0</added_mass_angular_jerk_limit>
    <added_mass_reset_scale_threshold>0.05</added_mass_reset_scale_threshold>

    <xU>-8.0</xU>
    <xUabsU>-15.0</xUabsU>
    <yV>-18.0</yV>
    <yVabsV>-65.0</yVabsV>
    <zW>-12.0</zW>
    <zWabsW>-42.0</zWabsW>
    <kP>-1.2</kP>
    <kPabsP>-1.5</kPabsP>
    <mQ>-4.0</mQ>
    <mQabsQ>-9.0</mQabsQ>
    <nR>-4.5</nR>
    <nRabsR>-10.5</nRabsR>

    <sample_point>0 0 0</sample_point>
  </hydrodynamics>
</plugin>
```

## SDF 参数说明

### 核心参数

| 参数 | 含义 |
|---|---|
| `link_name` | 接收流体动力作用的目标 link |
| `surface_z` | 全局水面高度 |
| `transition_band` | 水面附近连续过渡带的厚度 |
| `fluid_density_water` | 水体密度 |
| `fluid_density_air` | 空气密度 |
| `default_current` | 世界坐标系下的恒定流速向量 |
| `air_clearance_margin` | 空气推进器诊断所使用的净空阈值 |
| `sample_point` | 用于估算浸没程度的局部采样点 |
| `water_thruster_link_name` | 其原点浸没比会被发布为诊断值的 link |
| `air_prop_link_name` | 用于空气净空诊断检查的 link |

### 阻尼参数

插件采用与 Gazebo hydrodynamics 相同的系数命名风格，支持完整 6x6 线性项以及带 `abs` 的二次导数项。

常见的对角项包括：

- 线性项：`xU`、`yV`、`zW`、`kP`、`mQ`、`nR`
- 绝对值二次项：`xUabsU`、`yVabsV`、`zWabsW`、`kPabsP`、`mQabsQ`、`nRabsR`

如果按同样命名规则提供，交叉耦合项也会被解析。

像 `xUU` 这类已弃用、且不带 `abs` 的二次项会被忽略，同时产生警告。

### 附加质量参数

| 参数 | 含义 |
|---|---|
| `enable_added_mass` | 启用插件侧 `-M_A \hat{\dot{\nu}}_r` |
| `enable_added_mass_coriolis` | 启用简化的附加质量科里奥利/离心项 |
| `xDotU`、`yDotV`、`zDotW`、`kDotP`、`mDotQ`、`nDotR` | 按幅值解析的对角附加质量项 |

代码内部会将这 6 个附加质量参数存为绝对值，因此实现上统一按正幅值处理。

### 估计器参数

| 参数 | 含义 |
|---|---|
| `added_mass_velocity_filter_tau` | 机体系相对速度一阶低通滤波时间常数 |
| `added_mass_accel_filter_tau` | 差分加速度一阶低通滤波时间常数 |
| `added_mass_linear_accel_limit` | 线加速度裁剪上限 |
| `added_mass_angular_accel_limit` | 角加速度裁剪上限 |
| `added_mass_linear_jerk_limit` | 线 jerk 限幅 |
| `added_mass_angular_jerk_limit` | 角 jerk 限幅 |
| `added_mass_reset_scale_threshold` | 当 `mediumScale` 低于该阈值时重置估计器 |

## 计算模型

实现中的流体动力广义力写为：

$$
\tau_H = s(h, \rho_{eff}) \left( \tau_A + \tau_{CA} + \tau_D \right)
$$

其中：

- 机体系相对状态：$\nu_r = [u_r, v_r, w_r, p, q, r]^T$
- 阻尼项：$\tau_D = -D(\nu_r)\nu_r$
- 插件侧附加质量惯性项：$\tau_A = -M_A \hat{\dot{\nu}}_r$
- 简化附加质量耦合项：`tau_CA` 直接按照下文给出的叉乘表达式计算

### 相对速度状态

插件按以下路径构造状态量：

1. 读取 `WorldLinearVelocity`、`WorldAngularVelocity` 和 `WorldPose`
2. 计算 `relativeWorldLinear = worldLinear - default_current`
3. 将线速度和角速度旋转到机体系
4. 组装：

$$
\nu_r = [u_r, v_r, w_r, p, q, r]^T
$$

如果 `WorldPose` 或 `WorldLinearVelocity` 缺失，则本步会在返回前将估计器标记为陈旧并重置，同时不施加任何力。

### 过渡感知缩放

艇体浸没度通过配置的 `sample_point` 集合估算。对于每个采样点，代码会在 `transition_band` 范围内计算分段线性的浸没比例，再对所有采样点求平均。

有效密度与缩放因子定义为：

$$
\rho_{eff} = \rho_{air} + h(\rho_{water} - \rho_{air})
$$

$$
s(h, \rho_{eff}) = \mathrm{clamp}\left(h \cdot \frac{\rho_{eff}}{\rho_{water}}, 0, 1\right)
$$

同一个 `mediumScale` 会同时作用于阻尼、插件侧附加质量项，以及简化附加质量耦合项。

### 阻尼

阻尼矩阵由线性项和带 `abs` 的二次项共同组装：

$$
D_{ij}(\nu_r) = -L_{ij} - \sum_k Q_{ijk} |\nu_{r,k}|
$$

对应的机体系阻尼广义力为：

$$
\tau_D = -D(\nu_r)\nu_r
$$

### 附加质量估计器

代码并不直接对原始速度做差分，而是采用以下流程：

1. 对相对速度做一阶低通滤波
2. 由滤波后的速度做有限差分，得到加速度
3. 对加速度再做一阶低通滤波
4. 对加速度进行限幅裁剪
5. 对 jerk 进行限幅

最终得到估计值 $\hat{\dot{\nu}}_r$，并用于：

$$
\tau_A = -M_A \hat{\dot{\nu}}_r
$$

在以下情况下，估计器会被重置：仿真暂停、`dt` 非法、首次有效步，以及 `mediumScale` 过低。若缺少 `WorldPose` 或 `WorldLinearVelocity`，也会在返回前将其标记为陈旧并重置。

### 简化附加质量耦合项

实现中使用的是对角附加质量块：

$$
A_{11} = \mathrm{diag}(a_1, a_2, a_3), \qquad A_{22} = \mathrm{diag}(b_1, b_2, b_3)
$$

其中：

$$
\nu_1 = [u_r, v_r, w_r]^T, \qquad \nu_2 = [p, q, r]^T
$$

随后在机体系中计算：

- 力分量：`-Cross(A11 * nu1, nu2)`
- 力矩分量：`-Cross(A11 * nu1, nu1) - Cross(A22 * nu2, nu2)`

这是一种对角近似，并不是完整的 6x6 附加质量科里奥利矩阵。

惯性项仍然受到重置逻辑影响；而耦合项只要当前配置与所有权判定允许，就会基于当前状态实时计算。

## 过渡诊断信息

插件会在 `/model/<model_name>/transition_state` 上发布 `gz::msgs::Float_V`，内容包括：

1. 艇体浸没比例
2. 最多 4 个水下推进器 link 原点的浸没比例
3. 1 个空气净空标志位

这些值仅用于诊断，不会直接作为流体动力是否生效的门控条件。

## 与 `<fluid_added_mass>` 的交互关系

插件会检查目标 link 是否已经由引擎侧配置了非零的流体附加质量。

所有权检查会在 `Configure()` 阶段先执行一次。如果此时插件侧附加质量已启用，但尚未确认引擎侧所有权，那么 `PreUpdate(...)` 会在 `UpdateForcesAndMoments(...)` 执行前进行一次有界的运行时兜底复查。

如果 Gazebo 已经接管了 `<fluid_added_mass>`，插件会关闭自身的附加质量惯性项和简化附加质量耦合项，以避免重复计入。此时：

- 阻尼仍然生效
- 过渡诊断仍然生效
- 插件侧附加质量保持禁用

实际使用时，应在以下两种方案中二选一：

1. 使用引擎侧 `<fluid_added_mass>`
2. 使用插件侧对角附加质量加简化耦合项

两者同时启用会被有意阻止。

## 已知限制

这个插件属于工程近似模型，并不是完整的可变浸没流体动力求解器。

当前实现的主要限制包括：

- 仅支持对角附加质量
- 使用简化附加质量耦合项，而不是完整耦合形式
- 阻尼与附加质量相关项共用同一个 `mediumScale`
- 自由液面采用全局 `surface_z` 的平面假设
- 浸没度基于采样点平均，而不是基于几何体湿体积积分
- 仅支持平移型、恒定的世界系流场
- 仓库中暂未提供示例 world、测试套件或验证案例

当前的重置逻辑也默认仿真步进是正常连续的。如果你的工作流依赖瞬移、手动状态注入或非常大的 `dt`，那么附加质量估计器的行为应被视为对调参较敏感的近似处理。

## 当前实现的审阅备注

从现有代码审阅结果来看，主力学链路在内部是一致的，尤其是在坐标系处理、阻尼组装以及对 `<fluid_added_mass>` 冲突的保守规避方面表现较好。

当前最值得关注的点包括：

- 需要在代码注释和文档中统一简化附加质量耦合项的符号约定
- 附加质量所有权主要在 `Configure()` 阶段判定；如果该阶段无法确认，则运行时只会额外进行一次有界兜底复查
- 对惯性项的估计器重置策略是刻意保守的，而耦合项只要门控允许，就仍会对当前状态保持敏感

## 当前状态

目前这个仓库还是一个相对精简、独立的插件实现，暂时还不包含：

- 自动化测试
- 基准测试案例
- 示例 world 或载具模型
- 打包后的安装辅助工具

如果希望把它进一步完善成一个更完整、对外公开也更友好的 Gazebo 插件项目，这些内容会是很自然的下一步补充方向。
