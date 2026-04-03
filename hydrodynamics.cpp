/*
author: zyshine3
email: zyshine3@sjtu.edu.cn
institution: TeleAI,Chinatelecom,Shanghai
*/

#include "hydrodynamics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>

namespace {

constexpr std::size_t kStateSize = 6u;
constexpr double kAddedMassEpsilon = 1e-6;
constexpr double kReasonableTauEpsilon = 1e-9;
constexpr double kMaxReasonableDtSeconds = 0.1;

using Vec6 = std::array<double, kStateSize>;
using Mat6 = std::array<double, kStateSize * kStateSize>;

std::size_t MatrixIndex(const std::size_t _row, const std::size_t _col) {
  return _row * kStateSize + _col;
}

std::size_t TensorIndex(const std::size_t _i, const std::size_t _j,
                        const std::size_t _k) {
  return _i * kStateSize * kStateSize + _j * kStateSize + _k;
}

double SdfParamDouble(const sdf::ElementPtr &_element, const std::string &_field,
                     const double _defaultValue) {
  return _element->Get<double>(_field, _defaultValue).first;
}

void AddAngularVelocityComponents(const gz::sim::Entity &_entity,
                                  gz::sim::EntityComponentManager &_ecm) {
  if (!_ecm.Component<gz::sim::components::AngularVelocity>(_entity)) {
    _ecm.CreateComponent(_entity, gz::sim::components::AngularVelocity());
  }

  if (!_ecm.Component<gz::sim::components::WorldAngularVelocity>(_entity)) {
    _ecm.CreateComponent(_entity,
                         gz::sim::components::WorldAngularVelocity());
  }
}

void AddWorldPose(const gz::sim::Entity &_entity,
                  gz::sim::EntityComponentManager &_ecm) {
  if (!_ecm.Component<gz::sim::components::WorldPose>(_entity)) {
    _ecm.CreateComponent(_entity, gz::sim::components::WorldPose());
  }
}

void AddWorldLinearVelocity(const gz::sim::Entity &_entity,
                            gz::sim::EntityComponentManager &_ecm) {
  if (!_ecm.Component<gz::sim::components::WorldLinearVelocity>(_entity)) {
    _ecm.CreateComponent(_entity,
                         gz::sim::components::WorldLinearVelocity());
  }
}

Vec6 MatVecMul(const Mat6 &_matrix, const Vec6 &_vector) {
  Vec6 result{};
  for (std::size_t row = 0; row < kStateSize; ++row) {
    for (std::size_t col = 0; col < kStateSize; ++col) {
      result[row] += _matrix[MatrixIndex(row, col)] * _vector[col];
    }
  }
  return result;
}

bool HasNonZeroFluidAddedMass(const gz::math::Matrix6d &_matrix) {
  for (std::size_t row = 0; row < kStateSize; ++row) {
    for (std::size_t col = 0; col < kStateSize; ++col) {
      if (std::abs(_matrix(row, col)) > kAddedMassEpsilon) {
        return true;
      }
    }
  }
  return false;
}

bool HasNonZeroAddedMassDiagonal(const Vec6 &_diagonal) {
  for (const auto value : _diagonal) {
    if (std::abs(value) > kAddedMassEpsilon) {
      return true;
    }
  }
  return false;
}

double FirstOrderLpfSample(const double _input, const double _previous,
                           const double _dtSeconds, const double _tauSeconds) {
  if (_dtSeconds <= 0.0 || _tauSeconds <= kReasonableTauEpsilon) {
    return _input;
  }

  const double alpha = _dtSeconds / (_tauSeconds + _dtSeconds);
  return _previous + alpha * (_input - _previous);
}

Vec6 FirstOrderLpf(const Vec6 &_input, const Vec6 &_previous,
                   const double _dtSeconds, const double _tauSeconds) {
  Vec6 result{};
  for (std::size_t i = 0; i < kStateSize; ++i) {
    result[i] =
        FirstOrderLpfSample(_input[i], _previous[i], _dtSeconds, _tauSeconds);
  }
  return result;
}

Vec6 LimitAccelerationAmplitude(const Vec6 &_input,
                                const double _linearLimit,
                                const double _angularLimit) {
  Vec6 result = _input;
  for (std::size_t i = 0; i < kStateSize; ++i) {
    const double limit = i < 3u ? _linearLimit : _angularLimit;
    if (limit <= 0.0) {
      continue;
    }

    result[i] = std::clamp(result[i], -limit, limit);
  }
  return result;
}

Vec6 LimitAccelerationJerk(const Vec6 &_input, const Vec6 &_previous,
                           const double _dtSeconds,
                           const double _linearJerkLimit,
                           const double _angularJerkLimit) {
  if (_dtSeconds <= 0.0) {
    return _input;
  }

  Vec6 result = _input;
  for (std::size_t i = 0; i < kStateSize; ++i) {
    const double jerkLimit = i < 3u ? _linearJerkLimit : _angularJerkLimit;
    if (jerkLimit <= 0.0) {
      continue;
    }

    const double maxDelta = jerkLimit * _dtSeconds;
    const double delta =
        std::clamp(_input[i] - _previous[i], -maxDelta, maxDelta);
    result[i] = _previous[i] + delta;
  }
  return result;
}

gz::math::Vector3d Cross(const gz::math::Vector3d &_left,
                         const gz::math::Vector3d &_right) {
  return gz::math::Vector3d(
      _left.Y() * _right.Z() - _left.Z() * _right.Y(),
      _left.Z() * _right.X() - _left.X() * _right.Z(),
      _left.X() * _right.Y() - _left.Y() * _right.X());
}

std::string LinkNameOrEntity(const gz::sim::Link &_link,
                             const gz::sim::EntityComponentManager &_ecm) {
  const auto linkName = _link.Name(_ecm);
  if (linkName.has_value() && !linkName->empty()) {
    return *linkName;
  }

  return std::to_string(_link.Entity());
}

}  // namespace

GZ_ADD_PLUGIN(hydrodynamics::HydrodynamicsPlugin, gz::sim::System,
              hydrodynamics::HydrodynamicsPlugin::ISystemConfigure,
              hydrodynamics::HydrodynamicsPlugin::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(hydrodynamics::HydrodynamicsPlugin,
                    "zyshine3_gz_plugins::HybridHydrodynamics")

namespace hydrodynamics {

HydrodynamicsPlugin::HydrodynamicsPlugin() : System() {}
HydrodynamicsPlugin::~HydrodynamicsPlugin() {}

void HydrodynamicsPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    [[maybe_unused]] gz::sim::EventManager &_eventMgr) {
  this->model_ = gz::sim::Model(_entity);
  if (!this->model_.Valid(_ecm)) {
    gzerr << "[HybridHydrodynamics] Plugin can only be attached to a model."
          << std::endl;
    return;
  }

  this->ParseSdf(_sdf, _ecm);
  if (!this->link_.Valid(_ecm)) {
    gzerr << "[HybridHydrodynamics] Base link is invalid." << std::endl;
    return;
  }

  this->AddRequiredComponents(_ecm);
  this->DetectAddedMassOwnership(_ecm);

  this->transition_topic_ =
      "/model/" + this->model_.Name(_ecm) + "/transition_state";
  this->transition_publisher_ =
      this->node_.Advertise<gz::msgs::Float_V>(this->transition_topic_);
}

void HydrodynamicsPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
                                    gz::sim::EntityComponentManager &_ecm) {
  if (_info.paused) {
    this->reset_added_mass_estimator_ = true;
    return;
  }

  if (!this->link_.Valid(_ecm)) {
    return;
  }

  this->UpdateForcesAndMoments(_info, _ecm);
}

void HydrodynamicsPlugin::ParseSdf(
    const std::shared_ptr<const sdf::Element> &_sdf,
    const gz::sim::EntityComponentManager &_ecm) {
  if (!_sdf) {
    return;
  }

  auto sdfCopy = _sdf->Clone();
  if (sdfCopy->GetName() == "hydrodynamics") {
    this->ParseHydrodynamics(sdfCopy, _ecm);
    return;
  }

  for (sdf::ElementPtr element = sdfCopy->GetFirstElement(); element != nullptr;
       element = element->GetNextElement()) {
    if (element->GetName() == "hydrodynamics") {
      this->ParseHydrodynamics(element, _ecm);
    }
  }
}

void HydrodynamicsPlugin::ParseHydrodynamics(
    const sdf::ElementPtr _element,
    const gz::sim::EntityComponentManager &_ecm) {
  this->stability_linear_terms_.fill(0.0);
  this->stability_quadratic_abs_derivatives_.fill(0.0);
  this->added_mass_diagonal_.fill(0.0);
  this->engine_owns_added_mass_ = false;
  this->added_mass_estimator_initialized_ = false;
  this->reset_added_mass_estimator_ = true;
  this->filtered_relative_velocity_.fill(0.0);
  this->filtered_relative_acceleration_.fill(0.0);
  this->limited_relative_acceleration_.fill(0.0);

  std::string linkName;
  if (_element->HasElement("link_name")) {
    linkName = _element->Get<std::string>("link_name");
  } else {
    linkName = _element->Get<std::string>("link", "").first;
  }

  this->link_entity_ = this->model_.LinkByName(_ecm, linkName);
  this->link_ = gz::sim::Link(this->link_entity_);

  this->surface_z_ = _element->Get<double>("surface_z", this->surface_z_).first;
  this->transition_band_ =
      _element->Get<double>("transition_band", this->transition_band_).first;

  if (_element->HasElement("fluid_density_water")) {
    this->fluid_density_water_ = _element->Get<double>("fluid_density_water");
  } else if (_element->HasElement("water_density")) {
    this->fluid_density_water_ = _element->Get<double>("water_density");
  }

  this->fluid_density_air_ =
      _element->Get<double>("fluid_density_air", this->fluid_density_air_)
          .first;
  this->air_clearance_margin_ =
      _element->Get<double>("air_clearance_margin", this->air_clearance_margin_)
          .first;
  this->default_current_ =
      _element
          ->Get<gz::math::Vector3d>("default_current", this->default_current_)
          .first;

  this->enable_added_mass_ =
      _element->Get<bool>("enable_added_mass", this->enable_added_mass_).first;
  this->enable_added_mass_coriolis_ =
      _element
          ->Get<bool>("enable_added_mass_coriolis",
                      this->enable_added_mass_coriolis_)
          .first;
  this->added_mass_velocity_filter_tau_ =
      _element
          ->Get<double>("added_mass_velocity_filter_tau",
                        this->added_mass_velocity_filter_tau_)
          .first;
  this->added_mass_accel_filter_tau_ =
      _element
          ->Get<double>("added_mass_accel_filter_tau",
                        this->added_mass_accel_filter_tau_)
          .first;
  this->added_mass_linear_accel_limit_ =
      _element
          ->Get<double>("added_mass_linear_accel_limit",
                        this->added_mass_linear_accel_limit_)
          .first;
  this->added_mass_angular_accel_limit_ =
      _element
          ->Get<double>("added_mass_angular_accel_limit",
                        this->added_mass_angular_accel_limit_)
          .first;
  this->added_mass_linear_jerk_limit_ =
      _element
          ->Get<double>("added_mass_linear_jerk_limit",
                        this->added_mass_linear_jerk_limit_)
          .first;
  this->added_mass_angular_jerk_limit_ =
      _element
          ->Get<double>("added_mass_angular_jerk_limit",
                        this->added_mass_angular_jerk_limit_)
          .first;
  this->added_mass_reset_scale_threshold_ =
      _element
          ->Get<double>("added_mass_reset_scale_threshold",
                        this->added_mass_reset_scale_threshold_)
          .first;

  const std::array<std::string, kStateSize> addedMassFields = {
      "xDotU", "yDotV", "zDotW", "kDotP", "mDotQ", "nDotR"};
  for (std::size_t i = 0; i < kStateSize; ++i) {
    if (_element->HasElement(addedMassFields[i])) {
      this->added_mass_diagonal_[i] =
          std::abs(_element->Get<double>(addedMassFields[i]));
    }
  }

  this->WarnOnDeprecatedParameters(_element);

  const std::string snameConventionVel = "UVWPQR";
  const std::string snameConventionMoment = "xyzkmn";

  for (std::size_t i = 0; i < kStateSize; ++i) {
    for (std::size_t j = 0; j < kStateSize; ++j) {
      std::string prefix;
      prefix += snameConventionMoment[i];
      prefix += snameConventionVel[j];

      this->stability_linear_terms_[MatrixIndex(i, j)] =
          SdfParamDouble(_element, prefix, 0.0);

      for (std::size_t k = 0; k < kStateSize; ++k) {
        this->stability_quadratic_abs_derivatives_[TensorIndex(i, j, k)] =
            SdfParamDouble(_element, prefix + "abs" + snameConventionVel[k],
                           0.0);
      }
    }
  }

  this->sample_points_.clear();
  if (_element->HasElement("sample_point")) {
    for (auto pointElement = _element->GetElement("sample_point");
         pointElement != nullptr;
         pointElement = pointElement->GetNextElement("sample_point")) {
      this->sample_points_.push_back(pointElement->Get<gz::math::Vector3d>());
    }
  }

  if (this->sample_points_.empty()) {
    this->sample_points_.push_back(gz::math::Vector3d::Zero);
  }

  this->water_thruster_links_.clear();
  if (_element->HasElement("water_thruster_link_name")) {
    for (auto linkElement = _element->GetElement("water_thruster_link_name");
         linkElement != nullptr;
         linkElement = linkElement->GetNextElement("water_thruster_link_name")) {
      const auto name = linkElement->Get<std::string>();
      auto entity = this->model_.LinkByName(_ecm, name);
      if (entity != gz::sim::kNullEntity) {
        this->water_thruster_links_.emplace_back(entity);
      } else {
        gzerr << "[HybridHydrodynamics] Unknown water thruster link: " << name
              << std::endl;
      }
    }
  }

  this->air_prop_links_.clear();
  if (_element->HasElement("air_prop_link_name")) {
    for (auto linkElement = _element->GetElement("air_prop_link_name");
         linkElement != nullptr;
         linkElement = linkElement->GetNextElement("air_prop_link_name")) {
      const auto name = linkElement->Get<std::string>();
      auto entity = this->model_.LinkByName(_ecm, name);
      if (entity != gz::sim::kNullEntity) {
        this->air_prop_links_.emplace_back(entity);
      } else {
        gzerr << "[HybridHydrodynamics] Unknown air prop link: " << name
              << std::endl;
      }
    }
  }
}

void HydrodynamicsPlugin::AddRequiredComponents(
    gz::sim::EntityComponentManager &_ecm) {
  AddWorldPose(this->link_entity_, _ecm);
  AddAngularVelocityComponents(this->link_entity_, _ecm);
  AddWorldLinearVelocity(this->link_entity_, _ecm);
}

void HydrodynamicsPlugin::DetectAddedMassOwnership(
    gz::sim::EntityComponentManager &_ecm) {
  const auto linkName = LinkNameOrEntity(this->link_, _ecm);
  const auto worldFluidAddedMass = this->link_.WorldFluidAddedMassMatrix(_ecm);
  auto inertial = _ecm.Component<gz::sim::components::Inertial>(
      this->link_entity_);
  const bool hasWorldFluidAddedMass =
      worldFluidAddedMass.has_value() &&
      HasNonZeroFluidAddedMass(worldFluidAddedMass.value());
  const bool hasLocalFluidAddedMass =
      inertial != nullptr && inertial->Data().FluidAddedMass().has_value() &&
      HasNonZeroFluidAddedMass(inertial->Data().FluidAddedMass().value());
  const bool pluginAddedMassConfigured =
      HasNonZeroAddedMassDiagonal(this->added_mass_diagonal_) &&
      (this->enable_added_mass_ || this->enable_added_mass_coriolis_);

  if (hasWorldFluidAddedMass || hasLocalFluidAddedMass) {
    this->engine_owns_added_mass_ = true;
    if (pluginAddedMassConfigured) {
      gzwarn << "[HybridHydrodynamics] Detected non-zero <fluid_added_mass> "
                "on link ["
             << linkName
             << "]. This would double-count added mass, so plugin-side "
                "-M_A * nu_dot_r and C_A(nu_r)nu_r are disabled. Only "
                "damping and transition diagnostics remain active."
             << std::endl;
    } else {
      gzmsg << "[HybridHydrodynamics] Detected non-zero <fluid_added_mass> "
               "on link ["
            << linkName
            << "]. Plugin-side added mass remains disabled to avoid double "
               "counting."
            << std::endl;
    }
    return;
  }

  this->engine_owns_added_mass_ = false;
  if (pluginAddedMassConfigured) {
    gzmsg << "[HybridHydrodynamics] Plugin-side added mass is active on link ["
          << linkName << "] with inertia_term="
          << (this->enable_added_mass_ ? "on" : "off")
          << ", coriolis_term="
          << (this->enable_added_mass_coriolis_ ? "on" : "off") << "."
          << std::endl;
  } else {
    gzmsg << "[HybridHydrodynamics] Running damping-only hybrid "
             "hydrodynamics on link ["
          << linkName << "]." << std::endl;
  }
}

void HydrodynamicsPlugin::WarnOnDeprecatedParameters(
    const sdf::ElementPtr _element) const {
  const std::string snameConventionVel = "UVWPQR";
  const std::string snameConventionMoment = "xyzkmn";

  bool hasDeprecatedQuadratic = false;
  for (std::size_t i = 0; i < kStateSize && !hasDeprecatedQuadratic; ++i) {
    for (std::size_t j = 0; j < kStateSize && !hasDeprecatedQuadratic; ++j) {
      std::string prefix;
      prefix += snameConventionMoment[i];
      prefix += snameConventionVel[j];

      for (std::size_t k = 0; k < kStateSize; ++k) {
        const auto quadraticField = prefix + snameConventionVel[k];
        if (_element->HasElement(quadraticField)) {
          hasDeprecatedQuadratic = true;
          break;
        }
      }
    }
  }

  if (hasDeprecatedQuadratic) {
    gzwarn << "[HybridHydrodynamics] Non-abs quadratic damping parameters "
              "(such as <xUU>) are deprecated and ignored. Keep only "
              "<xUabsU>-style quadratic damping terms."
           << std::endl;
  }

  if (_element->HasElement("disable_added_mass") ||
      _element->HasElement("disable_coriolis")) {
    gzwarn << "[HybridHydrodynamics] <disable_added_mass> and "
              "<disable_coriolis> are deprecated and ignored. Use "
              "<enable_added_mass> and <enable_added_mass_coriolis> "
              "instead."
           << std::endl;
  }
}

void HydrodynamicsPlugin::UpdateForcesAndMoments(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) {
  const auto pose = this->link_.WorldPose(_ecm);
  auto worldLinear =
      _ecm.Component<gz::sim::components::WorldLinearVelocity>(
          this->link_entity_);
  const auto worldAngular =
      this->link_.WorldAngularVelocity(_ecm).value_or(gz::math::Vector3d::Zero);

  if (!pose.has_value() || worldLinear == nullptr) {
    return;
  }

  const auto relativeWorldLinear = worldLinear->Data() - this->default_current_;
  const auto linearLocal = pose->Rot().Inverse() * relativeWorldLinear;
  const auto angularLocal = pose->Rot().Inverse() * worldAngular;

  Vec6 state{};
  state[0] = linearLocal.X();
  state[1] = linearLocal.Y();
  state[2] = linearLocal.Z();
  state[3] = angularLocal.X();
  state[4] = angularLocal.Y();
  state[5] = angularLocal.Z();

  const double hullRatio =
      this->ComputeLinkSubmergence(this->link_, _ecm, this->sample_points_);

  std::vector<double> waterRatios;
  waterRatios.reserve(this->water_thruster_links_.size());
  for (const auto &thrusterLink : this->water_thruster_links_) {
    waterRatios.push_back(this->ComputeLinkOriginSubmergence(thrusterLink, _ecm));
  }
  while (waterRatios.size() < 4u) {
    waterRatios.push_back(0.0);
  }

  const bool airClearanceOk = this->ComputeAirClearance(_ecm);
  this->PublishTransitionState(hullRatio, waterRatios, airClearanceOk);

  const double effectiveDensity =
      this->fluid_density_air_ +
      hullRatio * (this->fluid_density_water_ - this->fluid_density_air_);
  const double densityScale =
      this->fluid_density_water_ > 1e-6
          ? effectiveDensity / this->fluid_density_water_
          : 1.0;
  const double mediumScale =
      std::clamp(hullRatio * densityScale, 0.0, 1.0);

  if (mediumScale < 1e-5) {
    this->reset_added_mass_estimator_ = true;
    return;
  }

  Mat6 dampingMatrix{};
  for (std::size_t i = 0; i < kStateSize; ++i) {
    for (std::size_t j = 0; j < kStateSize; ++j) {
      double coeff = -this->stability_linear_terms_[MatrixIndex(i, j)];
      for (std::size_t k = 0; k < kStateSize; ++k) {
        coeff -= this->stability_quadratic_abs_derivatives_[TensorIndex(i, j, k)] *
                 std::abs(state[k]);
      }
      dampingMatrix[MatrixIndex(i, j)] = coeff;
    }
  }

  const auto dampingWrench = MatVecMul(dampingMatrix, state);

  gz::math::Vector3d totalForce(-dampingWrench[0], -dampingWrench[1],
                                -dampingWrench[2]);
  gz::math::Vector3d totalTorque(-dampingWrench[3], -dampingWrench[4],
                                 -dampingWrench[5]);

  const bool pluginAddedMassEnabled =
      !this->engine_owns_added_mass_ &&
      HasNonZeroAddedMassDiagonal(this->added_mass_diagonal_) &&
      (this->enable_added_mass_ || this->enable_added_mass_coriolis_);

  if (pluginAddedMassEnabled) {
    const double dtSeconds =
        static_cast<double>(_info.dt.count()) / 1'000'000'000.0;
    const bool invalidDt =
        dtSeconds <= 0.0 || dtSeconds > kMaxReasonableDtSeconds;

    if (invalidDt || mediumScale < this->added_mass_reset_scale_threshold_ ||
        this->reset_added_mass_estimator_ ||
        !this->added_mass_estimator_initialized_) {
      this->ResetAddedMassEstimator(state);
    } else {
      const auto filteredVelocity =
          FirstOrderLpf(state, this->filtered_relative_velocity_, dtSeconds,
                        this->added_mass_velocity_filter_tau_);

      Vec6 rawAcceleration{};
      for (std::size_t i = 0; i < kStateSize; ++i) {
        rawAcceleration[i] =
            (filteredVelocity[i] - this->filtered_relative_velocity_[i]) /
            dtSeconds;
      }

      const auto filteredAcceleration =
          FirstOrderLpf(rawAcceleration, this->filtered_relative_acceleration_,
                        dtSeconds, this->added_mass_accel_filter_tau_);
      auto limitedAcceleration = LimitAccelerationAmplitude(
          filteredAcceleration, this->added_mass_linear_accel_limit_,
          this->added_mass_angular_accel_limit_);
      limitedAcceleration =
          LimitAccelerationJerk(limitedAcceleration,
                                this->limited_relative_acceleration_,
                                dtSeconds, this->added_mass_linear_jerk_limit_,
                                this->added_mass_angular_jerk_limit_);

      this->filtered_relative_velocity_ = filteredVelocity;
      this->filtered_relative_acceleration_ = filteredAcceleration;
      this->limited_relative_acceleration_ = limitedAcceleration;

      if (this->enable_added_mass_) {
        totalForce += gz::math::Vector3d(
            -this->added_mass_diagonal_[0] * limitedAcceleration[0],
            -this->added_mass_diagonal_[1] * limitedAcceleration[1],
            -this->added_mass_diagonal_[2] * limitedAcceleration[2]);
        totalTorque += gz::math::Vector3d(
            -this->added_mass_diagonal_[3] * limitedAcceleration[3],
            -this->added_mass_diagonal_[4] * limitedAcceleration[4],
            -this->added_mass_diagonal_[5] * limitedAcceleration[5]);
      }

      if (this->enable_added_mass_coriolis_) {
        const gz::math::Vector3d nu1(state[0], state[1], state[2]);
        const gz::math::Vector3d nu2(state[3], state[4], state[5]);
        const gz::math::Vector3d a11nu1(
            this->added_mass_diagonal_[0] * state[0],
            this->added_mass_diagonal_[1] * state[1],
            this->added_mass_diagonal_[2] * state[2]);
        const gz::math::Vector3d a22nu2(
            this->added_mass_diagonal_[3] * state[3],
            this->added_mass_diagonal_[4] * state[4],
            this->added_mass_diagonal_[5] * state[5]);

        totalForce += -Cross(a11nu1, nu2);
        totalTorque += -Cross(a11nu1, nu1) - Cross(a22nu2, nu2);
      }
    }
  }

  totalForce *= mediumScale;
  totalTorque *= mediumScale;

  this->link_.AddWorldWrench(_ecm, pose->Rot() * totalForce,
                             pose->Rot() * totalTorque);
}

double HydrodynamicsPlugin::SampleSubmergence(double _worldZ) const {
  if (this->transition_band_ <= 1e-6) {
    return _worldZ <= this->surface_z_ ? 1.0 : 0.0;
  }

  const double lower = this->surface_z_ - 0.5 * this->transition_band_;
  const double upper = this->surface_z_ + 0.5 * this->transition_band_;

  if (_worldZ <= lower) {
    return 1.0;
  }
  if (_worldZ >= upper) {
    return 0.0;
  }

  return (upper - _worldZ) / (upper - lower);
}

double HydrodynamicsPlugin::ComputeLinkSubmergence(
    const gz::sim::Link &_link, const gz::sim::EntityComponentManager &_ecm,
    const std::vector<gz::math::Vector3d> &_localSamplePoints) const {
  const auto pose = _link.WorldPose(_ecm);
  if (!pose.has_value()) {
    return 0.0;
  }

  if (_localSamplePoints.empty()) {
    return this->SampleSubmergence(pose->Pos().Z());
  }

  double ratio = 0.0;
  for (const auto &point : _localSamplePoints) {
    const auto worldPoint = pose->Pos() + pose->Rot().RotateVector(point);
    ratio += this->SampleSubmergence(worldPoint.Z());
  }

  return std::clamp(ratio / _localSamplePoints.size(), 0.0, 1.0);
}

double HydrodynamicsPlugin::ComputeLinkOriginSubmergence(
    const gz::sim::Link &_link,
    const gz::sim::EntityComponentManager &_ecm) const {
  const auto pose = _link.WorldPose(_ecm);
  if (!pose.has_value()) {
    return 0.0;
  }

  return this->SampleSubmergence(pose->Pos().Z());
}

bool HydrodynamicsPlugin::ComputeAirClearance(
    const gz::sim::EntityComponentManager &_ecm) const {
  if (this->air_prop_links_.empty()) {
    return true;
  }

  const double minAirZ = this->surface_z_ + this->air_clearance_margin_;
  for (const auto &airLink : this->air_prop_links_) {
    const auto pose = airLink.WorldPose(_ecm);
    if (!pose.has_value() || pose->Pos().Z() <= minAirZ) {
      return false;
    }
  }

  return true;
}

void HydrodynamicsPlugin::PublishTransitionState(
    double _hullRatio, const std::vector<double> &_waterRatios,
    bool _airClearanceOk) {
  if (!this->transition_publisher_) {
    return;
  }

  gz::msgs::Float_V msg;
  msg.add_data(static_cast<float>(_hullRatio));

  for (std::size_t i = 0; i < 4u; ++i) {
    const double value = i < _waterRatios.size() ? _waterRatios[i] : 0.0;
    msg.add_data(static_cast<float>(value));
  }

  msg.add_data(_airClearanceOk ? 1.0f : 0.0f);
  this->transition_publisher_.Publish(msg);
}

void HydrodynamicsPlugin::ResetAddedMassEstimator(
    const std::array<double, 6> &_state) {
  this->filtered_relative_velocity_ = _state;
  this->filtered_relative_acceleration_.fill(0.0);
  this->limited_relative_acceleration_.fill(0.0);
  this->added_mass_estimator_initialized_ = true;
  this->reset_added_mass_estimator_ = false;
}

}  // namespace hydrodynamics
