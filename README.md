# HybridHydrodynamics

`HybridHydrodynamics` is a standalone Gazebo Sim system plugin for underwater and air-water transition vehicles. It keeps the official Gazebo hydrodynamics velocity access pattern and `PreUpdate` lifecycle, then adds a transition-aware hydrodynamic model with damping, plugin-side diagonal added mass, a simplified added-mass Coriolis term, and transition-state diagnostics.

This repository currently contains one plugin implementation:

- plugin alias: `zyshine3_gz_plugins::HybridHydrodynamics`
- library target: `libhydrodynamics.so`
- source files: `hydrodynamics.cpp`, `hydrodynamics.hpp`

## What this plugin does

At each simulation step, the plugin:

1. reads link pose, world linear velocity, and world angular velocity
2. subtracts a constant world-frame current `default_current`
3. transforms the relative motion into the body frame
4. estimates hull submergence from `sample_point`
5. computes a continuous medium scaling factor `mediumScale`
6. assembles damping
7. optionally adds plugin-side added-mass inertia and a simplified added-mass Coriolis term when the current ownership/config gates allow them
8. rotates the final wrench back to the world frame and applies it with `AddWorldWrench`

The plugin also publishes transition diagnostics on:

- `/model/<model_name>/transition_state`

## Repository layout

```text
.
├── CMakeLists.txt
├── hydrodynamics.cpp
├── hydrodynamics.hpp
└── README.md
```

## Dependencies

The current `CMakeLists.txt` targets Gazebo Sim 8 and depends on:

- `gz-plugin2`
- `gz-sim8`
- `gz-transport13`
- `gz-math7`
- `gz-common5`
- `gz-msgs10`
- `Protobuf`

The plugin is built as C++17.

## Build

Build locally:

```bash
cmake -S . -B build
cmake --build build -j
```

The shared library is generated at:

```bash
build/libhydrodynamics.so
```

The install rule places it under:

```bash
lib/zyshine3_gz_plugins/
```

## Loading the plugin

Attach it to a model as a Gazebo Sim system plugin:

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

## SDF parameters

### Core parameters

| Parameter | Meaning |
|---|---|
| `link_name` | Target link that receives the hydrodynamic wrench |
| `surface_z` | Global water surface height |
| `transition_band` | Thickness of the continuous transition band around the surface |
| `fluid_density_water` | Water density |
| `fluid_density_air` | Air density |
| `default_current` | Constant current vector in the world frame |
| `air_clearance_margin` | Clearance threshold used by air-prop diagnostics |
| `sample_point` | Local points used to estimate hull submergence |
| `water_thruster_link_name` | Links whose origin submergence is published as diagnostics |
| `air_prop_link_name` | Links checked for air-clearance diagnostics |

### Damping parameters

The plugin parses the same coefficient naming style as Gazebo hydrodynamics, including full 6x6 linear terms and `abs`-quadratic derivatives.

Common diagonal terms are:

- linear: `xU`, `yV`, `zW`, `kP`, `mQ`, `nR`
- absolute quadratic: `xUabsU`, `yVabsV`, `zWabsW`, `kPabsP`, `mQabsQ`, `nRabsR`

Cross-coupled terms are also accepted if provided through the same naming pattern.

Deprecated non-`abs` quadratic terms such as `xUU` are ignored and produce a warning.

### Added-mass parameters

| Parameter | Meaning |
|---|---|
| `enable_added_mass` | Enables plugin-side `-M_A \hat{\dot{\nu}}_r` |
| `enable_added_mass_coriolis` | Enables the simplified added-mass Coriolis / centrifugal term |
| `xDotU`, `yDotV`, `zDotW`, `kDotP`, `mDotQ`, `nDotR` | Diagonal added-mass entries parsed by magnitude |

The code stores these six added-mass entries as absolute values, so the implementation treats them as positive magnitudes internally.

### Estimator parameters

| Parameter | Meaning |
|---|---|
| `added_mass_velocity_filter_tau` | LPF time constant for body-frame relative velocity |
| `added_mass_accel_filter_tau` | LPF time constant for finite-difference acceleration |
| `added_mass_linear_accel_limit` | Linear acceleration clipping limit |
| `added_mass_angular_accel_limit` | Angular acceleration clipping limit |
| `added_mass_linear_jerk_limit` | Linear jerk limit |
| `added_mass_angular_jerk_limit` | Angular jerk limit |
| `added_mass_reset_scale_threshold` | Resets the estimator when `mediumScale` drops below this value |

## Computation model

The implemented hydrodynamic wrench is:

$$
\tau_H = s(h, \rho_{eff}) \left( \tau_A + \tau_{CA} + \tau_D \right)
$$

with:

- body-frame relative state: $\nu_r = [u_r, v_r, w_r, p, q, r]^T$
- damping term: $\tau_D = -D(\nu_r)\nu_r$
- plugin-side added-mass inertia term: $\tau_A = -M_A \hat{\dot{\nu}}_r$
- simplified added-mass coupling term: `tau_CA` is implemented directly from the cross-product expressions shown below

### Relative velocity state

The plugin follows this path:

1. read `WorldLinearVelocity`, `WorldAngularVelocity`, and `WorldPose`
2. compute `relativeWorldLinear = worldLinear - default_current`
3. rotate linear and angular velocity into the body frame
4. assemble:

$$
\nu_r = [u_r, v_r, w_r, p, q, r]^T
$$

If `WorldPose` or `WorldLinearVelocity` is missing, the estimator is marked stale/reset before the function returns, and no force is applied for that step.

### Transition-aware scaling

Hull submergence is estimated from the configured `sample_point` set. For each point, the code evaluates a piecewise linear submergence ratio across `transition_band`, then averages the results.

The effective density and scaling factor are:

$$
\rho_{eff} = \rho_{air} + h(\rho_{water} - \rho_{air})
$$

$$
s(h, \rho_{eff}) = \mathrm{clamp}\left(h \cdot \frac{\rho_{eff}}{\rho_{water}}, 0, 1\right)
$$

The same `mediumScale` is applied to damping, plugin-side added mass, and the simplified added-mass coupling term.

### Damping

The damping matrix is assembled from linear and `abs`-quadratic terms:

$$
D_{ij}(\nu_r) = -L_{ij} - \sum_k Q_{ijk} |\nu_{r,k}|
$$

and the body-frame damping wrench is:

$$
\tau_D = -D(\nu_r)\nu_r
$$

### Added-mass estimator

The code does not use raw velocity differencing directly. Instead it applies:

1. first-order LPF on relative velocity
2. finite-difference acceleration from filtered velocity
3. first-order LPF on acceleration
4. acceleration clipping
5. jerk limiting

This produces the estimate $\hat{\dot{\nu}}_r$ used by:

$$
\tau_A = -M_A \hat{\dot{\nu}}_r
$$

The estimator is reset on pause, invalid `dt`, first valid step, and low-`mediumScale` conditions. Missing `WorldPose` / `WorldLinearVelocity` also marks it stale/reset before returning.

### Simplified added-mass coupling term

The implementation uses diagonal added-mass blocks:

$$
A_{11} = \mathrm{diag}(a_1, a_2, a_3), \qquad A_{22} = \mathrm{diag}(b_1, b_2, b_3)
$$

with:

$$
\nu_1 = [u_r, v_r, w_r]^T, \qquad \nu_2 = [p, q, r]^T
$$

The code then applies the body-frame terms:

- force part: `-Cross(A11 * nu1, nu2)`
- torque part: `-Cross(A11 * nu1, nu1) - Cross(A22 * nu2, nu2)`

This is a diagonal approximation, not a full 6x6 added-mass Coriolis matrix.

The inertia term is still reset-sensitive, but the coupling term is evaluated from the current state whenever plugin-side added mass is allowed by the ownership/config gates.

## Transition diagnostics

The plugin publishes `gz::msgs::Float_V` on `/model/<model_name>/transition_state` with:

1. hull submergence ratio
2. up to four water-thruster link-origin submergence ratios
3. one air-clearance flag

These values are diagnostics only. They do not directly gate the hydrodynamic wrench.

## Interaction with `<fluid_added_mass>`

The plugin checks whether the controlled link already has nonzero engine-side fluid added mass.

Ownership is checked in `Configure()`. If plugin-side added mass was configured but engine ownership was not confirmed there, `PreUpdate(...)` consumes one bounded runtime fallback re-check before `UpdateForcesAndMoments(...)` runs.

If Gazebo already owns `<fluid_added_mass>`, the plugin disables its own added-mass inertia and simplified added-mass coupling terms to avoid double counting. In that case:

- damping remains active
- transition diagnostics remain active
- plugin-side added mass stays disabled

In practice, you should choose one source of added mass:

1. engine-side `<fluid_added_mass>`, or
2. plugin-side diagonal added mass plus simplified coupling

Using both at the same time is intentionally blocked.

## Known limitations

This plugin is an engineering approximation, not a full variable-submergence hydrodynamic solver.

Current limitations in the implementation include:

- diagonal added mass only
- simplified added-mass coupling term instead of a full coupled formulation
- one shared `mediumScale` for damping and added-mass-related terms
- flat free-surface assumption using global `surface_z`
- sample-point averaging rather than geometry-based wetted-volume integration
- translational, constant, world-frame current only
- no in-repo example world, test suite, or validation case yet

The current reset logic also assumes normal simulation stepping. If your workflow relies on teleports, manual state injection, or very large `dt`, you should treat the added-mass estimator behavior as a tuning-sensitive approximation.

## Review notes from the current implementation

The current code review found that the main force pipeline is internally coherent, especially in frame handling, damping assembly, and conservative `<fluid_added_mass>` conflict avoidance.

The main points to watch are:

- the repository needs one unambiguous sign convention for the simplified added-mass coupling term in both code comments and documentation
- added-mass ownership is detected during `Configure()`, with one bounded runtime fallback re-check if the configure-time probe could not yet confirm engine ownership
- the estimator reset logic is intentionally conservative for inertia, while coupling remains current-state-sensitive when ownership/config gates allow it

## Status

At the moment this repository is a compact standalone plugin implementation. It does not yet include:

- automated tests
- benchmark cases
- example worlds or vehicle models
- packaged installation helpers

Those are the natural next additions if you want this repo to look and behave like a more complete public Gazebo plugin project.
