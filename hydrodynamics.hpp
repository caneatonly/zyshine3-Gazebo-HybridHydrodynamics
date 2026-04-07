#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include <gz/math.hh>
#include <gz/msgs/float_v.pb.h>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace hydrodynamics {

class HydrodynamicsPlugin : public gz::sim::System,
                            public gz::sim::ISystemConfigure,
                            public gz::sim::ISystemPreUpdate {
 public:
  HydrodynamicsPlugin();
  ~HydrodynamicsPlugin() override;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

 private:
  void ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf,
                const gz::sim::EntityComponentManager &_ecm);
  void ParseHydrodynamics(const sdf::ElementPtr _element,
                          const gz::sim::EntityComponentManager &_ecm);
  void AddRequiredComponents(gz::sim::EntityComponentManager &_ecm);
  void DetectAddedMassOwnership(gz::sim::EntityComponentManager &_ecm);
  void RecheckAddedMassOwnershipIfPending(
      gz::sim::EntityComponentManager &_ecm);
  void WarnOnDeprecatedParameters(const sdf::ElementPtr _element) const;
  void UpdateForcesAndMoments(const gz::sim::UpdateInfo &_info,
                              gz::sim::EntityComponentManager &_ecm);
  bool HasEngineOwnedAddedMass(
      gz::sim::EntityComponentManager &_ecm) const;
  bool HasPluginAddedMassConfigured() const;
  bool HasRequiredWorldState(
      const std::optional<gz::math::Pose3d> &_pose,
      const gz::sim::components::WorldLinearVelocity *_worldLinear) const;
  bool IsMediumScaleBelowHardCutoff(double _mediumScale) const;
  bool IsPluginAddedMassAllowed() const;
  bool ShouldResetAddedMassEstimator(const gz::sim::UpdateInfo &_info,
                                     double _mediumScale) const;
  bool IsInertiaTermAllowed(bool _pluginAddedMassDynamicsAllowed) const;
  bool IsCouplingTermAllowed(bool _pluginAddedMassAllowed) const;

  double SampleSubmergence(double _worldZ) const;
  double ComputeLinkSubmergence(
      const gz::sim::Link &_link,
      const gz::sim::EntityComponentManager &_ecm,
      const std::vector<gz::math::Vector3d> &_localSamplePoints) const;
  double ComputeLinkOriginSubmergence(
      const gz::sim::Link &_link,
      const gz::sim::EntityComponentManager &_ecm) const;
  bool ComputeAirClearance(const gz::sim::EntityComponentManager &_ecm) const;
  void PublishTransitionState(double _hullRatio,
                              const std::vector<double> &_waterRatios,
                              bool _airClearanceOk);

  gz::sim::Entity link_entity_{gz::sim::kNullEntity};
  gz::sim::Link link_{gz::sim::kNullEntity};
  gz::sim::Model model_{gz::sim::kNullEntity};

  std::vector<gz::math::Vector3d> sample_points_;
  std::vector<gz::sim::Link> water_thruster_links_;
  std::vector<gz::sim::Link> air_prop_links_;

  std::array<double, 6> added_mass_diagonal_{};
  std::array<double, 36> stability_linear_terms_{};
  std::array<double, 216> stability_quadratic_abs_derivatives_{};

  gz::math::Vector3d default_current_{0.0, 0.0, 0.0};

  double surface_z_{0.0};
  double transition_band_{0.2};
  double fluid_density_water_{1000.0};
  double fluid_density_air_{1.225};
  double air_clearance_margin_{0.05};

  bool enable_added_mass_{false};
  bool enable_added_mass_coriolis_{false};
  double added_mass_velocity_filter_tau_{0.03};
  double added_mass_accel_filter_tau_{0.06};
  double added_mass_linear_accel_limit_{4.0};
  double added_mass_angular_accel_limit_{8.0};
  double added_mass_linear_jerk_limit_{20.0};
  double added_mass_angular_jerk_limit_{40.0};
  double added_mass_reset_scale_threshold_{0.05};

  std::array<double, 6> filtered_relative_velocity_{};
  std::array<double, 6> filtered_relative_acceleration_{};
  std::array<double, 6> limited_relative_acceleration_{};
  bool added_mass_estimator_initialized_{false};
  bool reset_added_mass_estimator_{true};

  bool engine_owns_added_mass_{false};
  bool pending_added_mass_ownership_recheck_{false};

  gz::transport::Node node_;
  gz::transport::Node::Publisher transition_publisher_;
  std::string transition_topic_;

  void ResetAddedMassEstimator(const std::array<double, 6> &_state);
};

}  // namespace hydrodynamics
