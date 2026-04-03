# HybridHydrodynamics 插件说明
A Gazebo system plugin for implementing hydrodynamic effects on an air-water hybrid vehicle, developed based on Gazebo’s official Hydrodynamics plugin. 

## 依赖与版本

-  Ubuntu 22.04.4 LTS
- CMake `>= 3.10`
- GCC / Clang，支持 `C++17`
- `Gazebo Harmonic` 开发包

## Startup

### 1. Clone 仓库

```bash
git clone https://github.com/caneatonly/zyshine3-Gazebo-HybridHydrodynamics.git
cd zyshine3-Gazebo-HybridHydrodynamics
```

### 2. 配置并编译插件

```bash
cmake -S . -B build
cmake --build build -j
```

编译成功后，插件库通常位于：

```bash
build/libhydrodynamics.so
```

### 3. 让 Gazebo 能找到这个插件

运行仿真前，把插件目录加入 `GZ_SIM_SYSTEM_PLUGIN_PATH`：

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$(pwd)/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
```

### 4. 在模型或 world 的 SDF 中加载插件

示例：

```xml
<plugin filename="libhydrodynamics" name="zyshine3_gz_plugins::HybridHydrodynamics">
  <hydrodynamics>
    <link_name>base_link</link_name>
    <surface_z>0.0</surface_z>
    <transition_band>0.1</transition_band>
    <fluid_density_water>1000.0</fluid_density_water>
    <fluid_density_air>1.225</fluid_density_air>
    <sample_point>0 0 0</sample_point>
  </hydrodynamics>
</plugin>
```

## 插件功能

`HybridHydrodynamics` 是一个面向水空跨域飞行器的 Gazebo 系统插件。
当前主要实现的功能是：
- 基于官方 Gazebo hydrodynamics 的生命周期与取速路径实现阻尼项
- 计算相对水流速度
- 计算机体、涉水推进器、空气桨的跨介质诊断量
- 在水面附近对阻尼进行连续缩放，即通过计算机体入水比例来动态调整水阻尼的影响
- 发布 `/model/<name>/transition_state` 用于发布当前机体的浸水程度

它当前不负责：
- 插件内部附加质量
- 插件内部附加质量科氏项

其中浮力由世界级 `Buoyancy` 系统负责，推进器推力由独立的推进器插件负责，
附加质量由模型的 `<link><inertial><fluid_added_mass>` 交给 Gazebo 物理引擎求解。

## 基于 Fossen 方程的公式推导

### 完整 Fossen 水动力形式

Fossen 常见的 6 自由度水动力写法可记为：

$$
\tau_H = -M_A \dot{\nu}_r - C_A(\nu_r)\nu_r - D(\nu_r)\nu_r - g(\eta)
$$

其中：

- $\nu_r = [u_r, v_r, w_r, p, q, r]^T$ 是相对流体的机体系速度
- $M_A$ 是附加质量矩阵
- $C_A(\nu_r)$ 是附加质量对应的科氏与离心项
- $D(\nu_r)$ 是阻尼矩阵
- $g(\eta)$ 是恢复力项

对于当前插件，我们主动只保留阻尼项，把模型简化为：

$$
\tau_H = -D(\nu_r)\nu_r
$$

再结合跨水空介质的连续缩放，最终施加的水动力为：

$$
\tau_H^{applied} = s(h,\rho_{eff}) \left(-D(\nu_r)\nu_r\right)
$$

其中：

- $h$ 是机体浸没比例
- $\rho_{eff}$ 是水面附近的等效介质密度
- $s(h,\rho_{eff})$ 是连续缩放因子

### 只保留阻尼项的原因

这次设计有两个核心考虑：

1. `added mass` 已经通过 `<fluid_added_mass>` 交给 Gazebo 物理引擎求解，插件里再手工施加
   $-M_A \dot{\nu}_r$ 和对应科氏项会产生重复计算。
2. 原本通过速度差分显式构造加速度，再外加 `-M_A \dot{\nu}_r`，数值上容易形成显式反馈，
   在水面过渡、接触和高频步长下更容易触发发散或 ODE 崩溃。

## 当前实现流程

### 生命周期

插件沿用官方 Gazebo hydrodynamics 的骨架：

- `Configure`
- `PreUpdate`

### 速度获取路径
沿用官方取速方式：

- `WorldLinearVelocity`
- `WorldAngularVelocity`
- `WorldPose`

线速度先在世界系减去 `default_current`，得到相对水流的世界系速度；
再通过姿态旋转到机体系，得到：

$$
\nu_r = [u_r, v_r, w_r, p, q, r]^T
$$

### 浸没比例与连续缩放

插件通过多个 `sample_point` 计算机体浸没比例 `h`。
单点浸没函数采用水面附近线性过渡：

$$
\lambda(z)=
\begin{cases}
1, & z \le z_s-\frac{b}{2} \\
0, & z \ge z_s+\frac{b}{2} \\
\dfrac{z_s+\frac{b}{2}-z}{b}, & \text{otherwise}
\end{cases}
$$

其中：
- $z_s$ 对应 `surface_z`
- $b$ 对应 `transition_band`

机体浸没比例为各采样点平均值：

$$
h=\frac{1}{N}\sum_{i=1}^{N}\lambda(z_i)
$$

随后按空气密度与水密度插值得到等效密度：

$$
\rho_{eff} = \rho_{air} + h(\rho_{water}-\rho_{air})
$$

再计算连续缩放：

$$
s(h,\rho_{eff}) =
\mathrm{clamp}\left(
h \cdot \frac{\rho_{eff}}{\rho_{water}},
0, 1
\right)
$$

这个缩放只作用在阻尼项上，不作用于任何惯性项。

## 参数说明

### 插件有效参数

- `link_name`：确定水阻尼作用在哪一个Link上
- `surface_z`：确定水面高度，与world.sdf对齐
- `transition_band`：定义水空交界过渡区间，实现水空转换时的线性过渡
- `fluid_density_water` ：水密度
- `fluid_density_air`：空气密度
- `default_current`：洋流
- `air_clearance_margin`：用于确定空桨是否出水
- `sample_point`：采样点，用于计算浸沒比例
- `water_thruster_link_name`：水桨推进器名称
- `air_prop_link_name`：空桨推进器名称

阻尼参数仅保留两类：

- 线性项：`xU`, `yV`, `zW`, `kP`, `mQ`, `nR`
- `abs` 二次项：`xUabsU`, `yVabsV`, `zWabsW`, `kPabsP`, `mQabsQ`, `nRabsR`

## 附加质量策略

当前采用 `engine-first` 策略：

- 如果 link 上定义了非零 `<fluid_added_mass>`，则附加质量由 Gazebo physics 负责。
- 插件内部不再计算 `-M_A \dot{\nu}_r`。
- 插件内部不再计算附加质量科氏项。
- 如果没有定义非零 `<fluid_added_mass>`，插件退化为纯阻尼模式，并输出警告。
