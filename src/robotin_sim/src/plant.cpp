#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <optional>
#include <sstream>
#include <vector>

#include <gz/msgs/double.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace robotin
{
class Plant : public gz::sim::System,
              public gz::sim::ISystemConfigure,
              public gz::sim::ISystemPreUpdate,
              public gz::sim::ISystemPostUpdate,
              public gz::sim::ISystemReset
{
public:
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &config,
    gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
  {
    const gz::sim::World world(entity);
    const auto count = config->Get<unsigned int>("count");
    beltSpeed = config->Get<double>("belt_speed");
    if (config->HasElement("fixed_angle")) {
      fixedAngle = config->Get<double>("fixed_angle");
    }
    released.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
      const gz::sim::Model model(world.ModelByName(ecm, "fruit_" + std::to_string(i)));
      fruit.emplace_back(model.LinkByName(ecm, "body"));
    }
    for (std::size_t i = 0; i < cleats.size(); ++i) {
      cleats[i] = gz::sim::Model(world.ModelByName(ecm, "belt_cleat_" + std::to_string(i)));
    }
    const gz::sim::Model diverter(world.ModelByName(ecm, "diverter"));
    chute = gz::sim::Joint(diverter.JointByName(ecm, "yaw"));
    servo = gz::sim::Joint(diverter.JointByName(ecm, "servo_shaft"));
    rod = gz::sim::Joint(diverter.JointByName(ecm, "rod_hinge"));
    chute.EnablePositionCheck(ecm);
    servo.EnablePositionCheck(ecm);
    rod.EnablePositionCheck(ecm);

    const gz::sim::Model conveyor(world.ModelByName(ecm, "conveyor"));
    driveRoller = gz::sim::Joint(conveyor.JointByName(ecm, "drive_roller"));
    returnRoller = gz::sim::Joint(conveyor.JointByName(ecm, "return_roller"));
    driveRoller.EnableVelocityCheck(ecm);

    states = transport.Advertise<gz::msgs::StringMsg>("/robotin/plant");
    transport.Subscribe("/robotin/diverter/command", &Plant::ReceiveCommand, this);
  }

  void Reset(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &) override
  {
    started.store(false);
    target.store(0.0);
    std::fill(released.begin(), released.end(), false);
    beltTravel = 0.0;
    wasPaused = false;
    ++run;
  }

  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override
  {
    if (info.paused || !started.load()) {
      return;
    }

    const double step = std::chrono::duration<double>(info.dt).count();
    const auto servoPosition = servo.Position(ecm);
    if (servoPosition && !servoPosition->empty() && step > 0.0) {
      const double angle = servoPosition->front();
      const double nextAngle = angle + std::clamp(
        fixedAngle.value_or(target.load()) - angle, -servo_speed * step, servo_speed * step);
      MoveJoint(servo, nextAngle, step, ecm);
      MoveJoint(chute, nextAngle, step, ecm);
      MoveJoint(rod, -nextAngle, step, ecm);
    }

    driveRoller.SetVelocity(ecm, {beltSpeed / roller_radius});
    const auto driveVelocity = driveRoller.Velocity(ecm);
    const double rollerSpeed = driveVelocity && !driveVelocity->empty() ?
      driveVelocity->front() : 0.0;
    returnRoller.SetVelocity(ecm, {rollerSpeed});
    const double speed = rollerSpeed * roller_radius;
    beltTravel += speed * step;
    if (info.iterations % 5 == 0) {
      for (std::size_t i = 0; i < cleats.size(); ++i) {
        cleats[i].SetWorldPoseCmd(ecm, CleatPose(i));
      }
    }
    for (std::size_t i = 0; i < fruit.size(); ++i) {
      if (released[i]) {
        continue;
      }
      const auto pose = fruit[i].WorldPose(ecm);
      if (!pose) {
        continue;
      }
      if (pose->Pos().X() >= belt_edge) {
        released[i] = true;
        continue;
      }
      // Approximate traction using the measured tangential speed of the drive roller.
      fruit[i].SetLinearVelocity(ecm, pose->Rot().RotateVectorReverse({speed, 0, 0}));
      fruit[i].SetAngularVelocity(ecm, {0, 0, 0});
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
    const gz::sim::EntityComponentManager &ecm) override
  {
    const bool pauseChanged = info.paused != wasPaused;
    wasPaused = info.paused;
    if (!pauseChanged && (info.paused || info.iterations % 5 != 0)) {
      return;
    }
    const auto position = chute.Position(ecm);
    const auto servoPosition = servo.Position(ecm);
    if (!position || position->empty() || !servoPosition || servoPosition->empty()) {
      return;
    }
    std::ostringstream data;
    data << "{\"run\":" << run
         << ",\"paused\":" << (info.paused ? "true" : "false")
         << ",\"time\":" << std::chrono::duration<double>(info.simTime).count()
         << ",\"belt_speed\":" << beltSpeed
         << ",\"angle\":" << position->front()
         << ",\"servo\":" << servoPosition->front() << ",\"fruit\":[";
    for (std::size_t i = 0; i < fruit.size(); ++i) {
      const auto pose = fruit[i].WorldPose(ecm);
      if (!pose) {
        return;
      }
      const auto &p = pose->Pos();
      data << (i == 0 ? "" : ",") << '[' << p.X() << ',' << p.Y() << ',' << p.Z() << ']';
    }
    data << "]}";
    gz::msgs::StringMsg message;
    message.set_data(data.str());
    states.Publish(message);
  }

private:
  gz::math::Pose3d CleatPose(std::size_t index) const
  {
    constexpr double pi = 3.141592653589793;
    constexpr double arcRadius = 0.028;
    constexpr double straight = 0.450;
    constexpr double halfTurn = pi * arcRadius;
    constexpr double track = 2 * (straight + halfTurn);
    double offset = std::fmod(beltTravel + index * track / cleats.size(), track);
    if (offset < straight) {
      return {0.025 + offset, 0, 0.403, 0, 0, 0};
    }
    offset -= straight;
    if (offset < halfTurn) {
      const double angle = offset / arcRadius;
      return {0.475 + arcRadius * std::sin(angle), 0, 0.375 + arcRadius * std::cos(angle),
        0, angle, 0};
    }
    offset -= halfTurn;
    if (offset < straight) {
      return {0.475 - offset, 0, 0.347, 0, pi, 0};
    }
    const double angle = pi + (offset - straight) / arcRadius;
    return {0.025 + arcRadius * std::sin(angle), 0, 0.375 + arcRadius * std::cos(angle),
      0, angle, 0};
  }

  static void MoveJoint(gz::sim::Joint &joint, double target, double step,
    gz::sim::EntityComponentManager &ecm)
  {
    const auto position = joint.Position(ecm);
    if (position && !position->empty()) {
      const double speed = (target - position->front()) / step;
      joint.SetVelocity(ecm, {std::clamp(speed, -7.0, 7.0)});
    }
  }

  void ReceiveCommand(const gz::msgs::Double &message)
  {
    if (std::isfinite(message.data())) {
      target.store(std::clamp(message.data(), -0.76, 0.76));
      started.store(true);
    }
  }

  static constexpr double belt_edge = 0.5;
  static constexpr double roller_radius = 0.025;
  static constexpr double servo_speed = 5.77322;  // 86 degrees in 260 ms.
  double beltSpeed = 0.1;
  double beltTravel = 0.0;
  unsigned int run = 0;
  bool wasPaused = false;
  std::optional<double> fixedAngle;
  std::vector<gz::sim::Link> fruit;
  std::vector<bool> released;
  std::array<gz::sim::Model, 7> cleats;
  gz::sim::Joint chute;
  gz::sim::Joint servo;
  gz::sim::Joint rod;
  gz::sim::Joint driveRoller;
  gz::sim::Joint returnRoller;
  std::atomic<double> target{0.0};
  std::atomic<bool> started{false};
  gz::transport::Node transport;
  gz::transport::Node::Publisher states;
};
}  // namespace robotin

GZ_ADD_PLUGIN(robotin::Plant, gz::sim::System, gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate, gz::sim::ISystemPostUpdate, gz::sim::ISystemReset)
