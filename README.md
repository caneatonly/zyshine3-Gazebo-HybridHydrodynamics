# HybridHydrodynamics v2 说明

`HybridHydrodynamics` 是一个面向水空跨域平台的 Gazebo System 插件。
它以 Gazebo 官方 hydrodynamics 的取速路径和 `PreUpdate` 生命周期为骨架，在其上增加了：

- 水空跨域浸没比例计算
- 连续介质缩放 `mediumScale`
- 插件侧对角附加质量 `-M_A \hat{\dot{\nu}}_r`
- 主对角近似的附加质量科氏项 `-C_A(\nu_r)\nu_r`
- `/model/<name>/transition_state` 诊断发布

当前版本只维护 `external/plugins/hydrodynamics` 这份外部插件实现。

## 1. 当前动力学形式

本插件当前施加的水动力写成：

$$
\tau_H =
s(h,\rho_{eff})
\left(
-M_A \hat{\dot{\nu}}_r
-C_A(\nu_r)\nu_r
-D(\nu_r)\nu_r
\right)
$$

其中：

- $\nu_r = [u_r, v_r, w_r, p, q, r]^T$：机体系相对流体速度
- $M_A$：仅保留主对角项的附加质量矩阵
- $C_A(\nu_r)\nu_r$：由主对角附加质量导出的简化科氏/离心项
- $D(\nu_r)\nu_r$：阻尼项
- $s(h,\rho_{eff})$：由浸没比例与等效密度计算出的连续缩放

这里的 $\hat{\dot{\nu}}_r$ 不是直接差分速度，而是经过滤波、加速度限幅和 jerk 限制后的估计值。

## 2. 为什么这样实现

直接用速度差分构造插件侧 `-M_A \dot{\nu}_r` 很容易在以下场景引入尖峰：

- 模型刚刚 spawn
- pause / resume
- 水面穿越
- 接触求解脉冲
- 数值噪声导致的高频速度跳变

因此当前版本采用“工程近似的稳化估计器”：

1. 先对相对速度做一阶低通
2. 再差分得到原始加速度
3. 对原始加速度再做一阶低通
4. 分别对线加速度和角加速度做幅值限制
5. 再分别做 jerk 限制
6. 在空气段、缩放很小、首次有效步、pause/resume 后第一拍等条件下复位估计器

这不是严格的变浸没附加质量动量模型，但能在跨域仿真里提供更稳定、可调的附加质量近似。

## 3. 速度与加速度估计

插件沿用官方 Gazebo hydrodynamics 的速度获取路径：

- `WorldLinearVelocity`
- `WorldAngularVelocity`
- `WorldPose`

处理流程为：

1. 世界系线速度减去 `default_current`
2. 用姿态旋转到机体系
3. 构造

$$
\nu_r = [u_r, v_r, w_r, p, q, r]^T
$$

附加质量项里的加速度估计为：

$$
\nu_f(k) = \mathrm{LPF}(\nu_r(k))
$$

$$
\dot{\nu}_{raw}(k) = \frac{\nu_f(k)-\nu_f(k-1)}{\Delta t}
$$

$$
\dot{\nu}_f(k) = \mathrm{LPF}(\dot{\nu}_{raw}(k))
$$

$$
\hat{\dot{\nu}}_r(k) =
\mathrm{JerkLimit}
\left(
\mathrm{AccelClip}(\dot{\nu}_f(k))
\right)
$$

最终使用：

$$
\tau_A = -M_A \hat{\dot{\nu}}_r
$$

## 4. 主对角附加质量与 `C_A(\nu_r)\nu_r`

### 4.1 参数约定

SDF 中仍沿用传统导数写法：

```xml
<xDotU>-2.4</xDotU>
<yDotV>-5.2</yDotV>
<zDotW>-2.1</zDotW>
<kDotP>-0.08</kDotP>
<mDotQ>-0.24</mDotQ>
<nDotR>-0.32</nDotR>
```

插件内部按正幅值使用：

$$
a_1 = -xDotU,\quad a_2 = -yDotV,\quad a_3 = -zDotW
$$

$$
b_1 = -kDotP,\quad b_2 = -mDotQ,\quad b_3 = -nDotR
$$

并构造：

$$
A_{11}=\mathrm{diag}(a_1,a_2,a_3),\qquad
A_{22}=\mathrm{diag}(b_1,b_2,b_3)
$$

### 4.2 简化 `C_A(\nu_r)\nu_r`

把 6 维速度拆成：

$$
\nu_1 = [u_r,v_r,w_r]^T,\qquad
\nu_2 = [p,q,r]^T
$$

当前实现采用主对角近似：

$$
C_A(\nu_r)\nu_r =
\begin{bmatrix}
-S(A_{11}\nu_1)\nu_2 \\
-S(A_{11}\nu_1)\nu_1 - S(A_{22}\nu_2)\nu_2
\end{bmatrix}
$$

其中叉乘矩阵满足：

$$
S(x)y = x \times y
$$

也就是说，当前实现里：

- 平动力部分：`-Cross(A11 * nu1, nu2)`
- 力矩部分：`-Cross(A11 * nu1, nu1) - Cross(A22 * nu2, nu2)`

这部分不依赖加速度差分，通常比裸的插件侧 `-M_A \dot{\nu}_r` 更稳定。

## 5. 阻尼项

阻尼仍保持当前的“线性项 + abs 二次项”形式：

$$
\tau_D = -D(\nu_r)\nu_r
$$

当前只保留：

- 线性项：`xU`, `yV`, `zW`, `kP`, `mQ`, `nR`
- `abs` 二次项：`xUabsU`, `yVabsV`, `zWabsW`, `kPabsP`, `mQabsQ`, `nRabsR`

不再使用 `xUU` 这类非 `abs` 二次项。

## 6. 水空跨域缩放

插件通过多个 `sample_point` 估计机体浸没比例 `h`，再结合空气/水密度计算：

$$
\rho_{eff} = \rho_{air} + h(\rho_{water}-\rho_{air})
$$

$$
s(h,\rho_{eff}) =
\mathrm{clamp}
\left(
h \cdot \frac{\rho_{eff}}{\rho_{water}},
0, 1
\right)
$$

当前 `-M_A \hat{\dot{\nu}}_r`、`-C_A(\nu_r)\nu_r` 和阻尼项都乘同一个 `mediumScale`。

这是一种跨域工程近似。当前版本没有显式建模：

$$
\frac{d}{dt}(M_A(\lambda)\nu_r)
$$

里与 `\dot{\lambda}` 相关的附加项。

## 7. 与 `<fluid_added_mass>` 的关系

当前版本由**插件侧附加质量主导**。

因此：

- `teleh4z/model.sdf` 中的 `<fluid_added_mass>` 必须继续保持注释
- 如果 link 上检测到非零 `<fluid_added_mass>`，插件会发出高优先级警告
- 为避免双重计算，插件会安全回退为“只保留阻尼与诊断”的模式

换句话说，下面两种路径只能选一种：

1. 物理引擎 `<fluid_added_mass>`
2. 插件侧 `-M_A \hat{\dot{\nu}}_r + C_A(\nu_r)\nu_r`

当前 v2 选择的是第 2 种。

## 8. 估计器复位条件

插件会在以下情况下复位附加质量估计器，并在该拍不施加插件侧 added mass / `C_A`：

- `dt <= 0`
- `dt` 明显异常
- `mediumScale < added_mass_reset_scale_threshold`
- 第一次进入有效步
- pause / resume 后第一拍

这么做是为了避免：

- 刚入水瞬间的伪尖峰
- 刚 spawn 或 pause/resume 带来的虚假差分加速度

## 9. 主要 SDF 参数

### 9.1 基础参数

- `link_name`
- `surface_z`
- `transition_band`
- `fluid_density_water`
- `fluid_density_air`
- `default_current`
- `air_clearance_margin`
- `sample_point`
- `water_thruster_link_name`
- `air_prop_link_name`

### 9.2 附加质量与科氏项

- `enable_added_mass`
- `enable_added_mass_coriolis`
- `xDotU`
- `yDotV`
- `zDotW`
- `kDotP`
- `mDotQ`
- `nDotR`

### 9.3 稳化估计器参数

- `added_mass_velocity_filter_tau`
- `added_mass_accel_filter_tau`
- `added_mass_linear_accel_limit`
- `added_mass_angular_accel_limit`
- `added_mass_linear_jerk_limit`
- `added_mass_angular_jerk_limit`
- `added_mass_reset_scale_threshold`

## 10. teleh4z 当前示例

```xml
<plugin filename="/home/user/external/plugins/hydrodynamics/build/libhydrodynamics.so"
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
  </hydrodynamics>
</plugin>
```

## 11. 已知局限

- 这是跨域工程近似，不是严格的变质量流体动量求解
- `mediumScale` 对附加质量项和 `C_A` 的连续缩放在物理上只是近似
- 参数需要结合机体质量、浮力裕度、推进器动态和仿真步长调参
- 若 heave 通道的 `zDotW` 过大，或滤波/限幅过松，可能出现“上浮被压制”这类非理想现象

## 12. 编译

```bash
cd /home/user/external/plugins/hydrodynamics
cmake -S . -B build
cmake --build build -j
```

生成库文件：

```bash
/home/user/external/plugins/hydrodynamics/build/libhydrodynamics.so
```
