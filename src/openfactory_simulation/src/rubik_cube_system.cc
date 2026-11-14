// Mechanism of the Rubik's cube model (phase 7, D-022, D-023).
//
// The model (scripts/rubik_model.py) is a core, six axles on revolute joints about the face
// normals, and 26 cubies that are free links. This system welds every cubie to the body its
// layer turns with, and changes those welds when the forces on the cube ask for another axis:
//
//   - One axis is active at a time. Its two outer layers are welded to their axles, the
//     middle slab to the core. The two axles turn under whatever torque reaches them, against
//     the joint friction of the model and a detent that pulls a layer square when it is close
//     to a quarter turn (a magnetic cube). The other four axles carry nothing and are held.
//   - The torque that the outside world applies to each layer of the other two axes is read
//     from the wrench every weld transmits (a cubie is a leaf: what its weld carries is what
//     pushes on it). When it exceeds the breakaway torque and both active layers are square
//     within `snap_tolerance`, where the detent leaves a layer that nothing holds, their
//     residual angle is set to exactly zero and the welds are rebuilt for the new axis.
//     While an active layer is out of square the other axes stay rigid, as on the real cube.
//
// Nothing here moves a cubie: layers turn only under forces applied to the links. What is
// idealised is inside the cube: no interlocking pieces, and the exact squaring of a layer
// (at most `snap_tolerance`) before an axis change, without which the error of every turn
// would add up in the welds.
//
// Welds are DetachableJoint entities, which the Physics system turns into joints of the
// model tree. Physics creates new ones before a step and removes old ones after it, so the
// welds are removed in one update and created in the next with no step in between. An axis
// change takes three updates: square exactly; remove the welds; create the new ones.
//
// The two layers that were free must not move during those steps, and they are often loaded:
// what asks for the new axis (a gripper turning the cube against the nest) pushes on them too.
// So while the axis changes each of them is held by a torque equal and opposite to the one
// the outside applies to it, read from its welds like the demand; if it moves all the same,
// what moved it is worked out from how far it went in one step from rest, added to the hold,
// and the squaring is tried again. The hold is what the pieces of a real cube do to a layer
// once another one has started to turn. If a layer still moves when the welds are already
// gone, the old welds are put back.

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

namespace openfactory
{
using gz::math::Quaterniond;
using gz::math::Vector3d;
using gz::sim::Entity;
using gz::sim::EntityComponentManager;
using gz::sim::kNullEntity;
namespace components = gz::sim::components;

constexpr double kQuarter = M_PI / 2.0;
// A layer that was set square and has been held since is within this angle of it. It is
// what a change of axis may leave in the welds: 2 urad is 0.05 um at a corner.
constexpr double kStill = 2e-6;
// Squarings tried in a row for one change of axis before the cube is left as it is.
constexpr int kMaxAttempts = 50;
// Share of the estimated torque added to the hold at every attempt, and the cap of the hold.
constexpr double kHoldGain = 0.9;
constexpr double kMaxHold = 2.0;
// Steps in a row for which another axis must be asked for before it engages: a knock (the
// cube landing, the fingers closing) does not change the axis, a push does.
constexpr int kPersistence = 25;

// Axle k turns the layer on side (k % 2 ? -1 : +1) of axis k / 2 of the cube frame
// (x towards R, y towards U, z towards F).
constexpr std::array<const char *, 6> kFaces{"R", "L", "U", "D", "F", "B"};

struct Axle
{
  Entity link{kNullEntity};
  Entity joint{kNullEntity};
  double angle{0.0};
};

struct Cubie
{
  Entity link{kNullEntity};
  Entity weld{kNullEntity};
  std::array<int, 3> slot{0, 0, 0};
  Vector3d position;          // in the core frame
  Quaterniond rotation;       // cubie frame to core frame
};

enum class State { kActive, kDetach, kAttach };

class RubikCubeSystem : public gz::sim::System,
                        public gz::sim::ISystemConfigure,
                        public gz::sim::ISystemPreUpdate,
                        public gz::sim::ISystemPostUpdate
{
  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm, gz::sim::EventManager &) override
  {
    gz::sim::Model model(_entity);
    auto param = [&](const char *_name, double _default)
    {
      return _sdf->Get<double>(_name, _default).first;
    };
    this->pitch = param("pitch", this->pitch);
    this->breakaway = param("breakaway_torque", this->breakaway);
    this->detentTorque = param("detent_torque", this->detentTorque);
    this->detentStiffness = param("detent_stiffness", this->detentStiffness);
    this->detentWindow = param("detent_window", this->detentWindow);
    this->snapTolerance = param("snap_tolerance", this->snapTolerance);
    this->layerInertia = param("layer_inertia", this->layerInertia);
    this->activeAxis = _sdf->Get<int>("initial_axis", this->activeAxis).first;

    this->core = model.LinkByName(_ecm, "core");
    bool found = this->core != kNullEntity;
    for (std::size_t k = 0; k < 6; ++k)
    {
      const std::string name = std::string("axle_") + kFaces[k];
      this->axles[k].link = model.LinkByName(_ecm, name);
      this->axles[k].joint = model.JointByName(_ecm, name + "_joint");
      found = found && this->axles[k].link != kNullEntity &&
              this->axles[k].joint != kNullEntity;
    }
    for (const Entity link : model.Links(_ecm))
    {
      auto name = _ecm.Component<components::Name>(link);
      if (name && name->Data().rfind("cubie_", 0) == 0)
      {
        Cubie cubie;
        cubie.link = link;
        this->cubies.push_back(cubie);
      }
    }
    if (!found || this->cubies.size() != 26 || this->activeAxis < 0 || this->activeAxis > 2)
    {
      gzerr << "RubikCubeSystem: model [" << model.Name(_ecm) << "] needs links core, "
            << "axle_<R|L|U|D|F|B> with their joints and 26 cubie_* links, and initial_axis "
            << "0, 1 or 2; found " << this->cubies.size() << " cubies. Mechanism disabled."
            << std::endl;
      return;
    }
    for (auto &axle : this->axles)
    {
      _ecm.CreateComponent(axle.joint, components::JointPosition());
    }
    this->publisher = this->node.Advertise<gz::msgs::StringMsg>(
        "/model/" + model.Name(_ecm) + "/mechanism");
    this->configured = true;
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                         EntityComponentManager &_ecm) override
  {
    if (!this->configured || _info.paused)
    {
      return;
    }
    const double dt = std::chrono::duration<double>(_info.dt).count();
    this->ReadState(_ecm);

    if (!this->welded)
    {
      // First update: the welds exist before the first step, so nothing falls apart.
      this->ReadSlots();
      this->CreateWelds(_ecm);
      this->welded = true;
      this->settle = 3;
    }

    // Every axle is force-controlled: the active ones feel the detent, the others carry
    // nothing and stay where the joint friction of the model leaves them.
    const int demanded = this->Demand(_ecm);
    bool square = true;
    bool still = true;
    std::array<double, 2> error{0.0, 0.0};
    for (int side = 0; side < 2; ++side)
    {
      error[side] = Error(this->axles[2 * this->activeAxis + side]);
      square = square && std::abs(error[side]) <= this->snapTolerance;
      still = still && std::abs(error[side]) <= kStill;
    }
    if (this->state == State::kDetach && !still)
    {
      // A free layer moved in the step after it was squared: from rest, under a torque t,
      // it turns t dt^2 / inertia in one step. Hold against that torque and square again.
      for (int side = 0; side < 2; ++side)
      {
        this->hold[side] = std::clamp(
            this->hold[side] - kHoldGain * this->layerInertia * error[side] / (dt * dt),
            -kMaxHold, kMaxHold);
      }
    }
    for (std::size_t k = 0; k < 6; ++k)
    {
      double torque = 0.0;
      if (static_cast<int>(k / 2) == this->activeAxis)
      {
        const int side = static_cast<int>(k % 2);
        if (std::abs(error[side]) < this->detentWindow)
        {
          torque = std::clamp(-this->detentStiffness * error[side], -this->detentTorque,
                              this->detentTorque);
        }
        if (this->state != State::kActive)
        {
          // the joint turns about the outward normal: minus the cube axis on side 1
          torque += this->hold[side] +
                    (side == 0 ? 1.0 : -1.0) * this->moment[this->activeAxis][side];
        }
      }
      _ecm.SetComponentData<components::JointForceCmd>(this->axles[k].joint, {torque});
    }

    switch (this->state)
    {
      case State::kActive:
      {
        this->blocked = false;
        if (this->settle > 0)
        {
          --this->settle;
          break;
        }
        const int axis = demanded;
        this->asked = axis >= 0 && axis == this->askedAxis ? this->asked + 1 : 0;
        this->askedAxis = axis;
        if (axis < 0 || this->asked < kPersistence)
        {
          break;
        }
        if (!square)
        {
          this->blocked = true;
          break;
        }
        // Both active layers are within the tolerance: square them exactly. The axles of
        // the new axis carry nothing and start from zero.
        this->targetAxis = axis;
        this->attempts = 0;
        this->Square(_ecm);
        this->state = State::kDetach;
        break;
      }
      case State::kDetach:
        if (!still)
        {
          if (!square || ++this->attempts > kMaxAttempts)
          {
            // pushed out of the tolerance, or never at rest: the cube stays as it is
            this->hold = {0.0, 0.0};
            this->blocked = true;
            this->state = State::kActive;
            this->settle = 3;
            break;
          }
          this->Square(_ecm);
          break;
        }
        for (auto &cubie : this->cubies)
        {
          _ecm.RequestRemoveEntity(cubie.weld);
          cubie.weld = kNullEntity;
        }
        this->state = State::kAttach;
        break;
      case State::kAttach:
        this->ReadSlots();
        if (still)
        {
          this->activeAxis = this->targetAxis;
          ++this->switches;
        }
        else
        {
          // the cubies moved with their old parents until the welds went: put them back
          ++this->aborted;
        }
        this->CreateWelds(_ecm);
        this->hold = {0.0, 0.0};
        this->state = State::kActive;
        this->settle = 3;
        break;
    }
  }

  public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                          const EntityComponentManager &) override
  {
    if (!this->configured || _info.paused || _info.iterations % 20 != 0)
    {
      return;
    }
    static const char *names[] = {"active", "detach", "attach"};
    std::ostringstream out;
    out.precision(9);
    out << "{\"sim_time\": " << std::chrono::duration<double>(_info.simTime).count()
        << ", \"axis\": " << this->activeAxis << ", \"state\": \""
        << names[static_cast<int>(this->state)] << "\", \"switches\": " << this->switches
        << ", \"aborted\": " << this->aborted << ", \"attempts\": " << this->attempts
        << ", \"blocked\": " << (this->blocked ? "true" : "false") << ", \"angles\": [";
    for (std::size_t k = 0; k < 6; ++k)
    {
      out << (k ? ", " : "") << this->axles[k].angle;
    }
    out << "], \"demand\": [" << this->demand[0] << ", " << this->demand[1] << ", "
        << this->demand[2] << "]}";
    gz::msgs::StringMsg msg;
    msg.set_data(out.str());
    this->publisher.Publish(msg);
  }

  /// Axle angles, and the pose of every cubie in the core frame.
  private: void ReadState(const EntityComponentManager &_ecm)
  {
    for (auto &axle : this->axles)
    {
      auto position = _ecm.Component<components::JointPosition>(axle.joint);
      if (position && !position->Data().empty())
      {
        axle.angle = position->Data()[0];
      }
    }
    const auto corePose = gz::sim::worldPose(this->core, _ecm);
    for (auto &cubie : this->cubies)
    {
      const auto pose = gz::sim::worldPose(cubie.link, _ecm);
      cubie.position = corePose.Rot().RotateVectorReverse(pose.Pos() - corePose.Pos());
      cubie.rotation = corePose.Rot().Inverse() * pose.Rot();
    }
  }

  private: void ReadSlots()
  {
    for (auto &cubie : this->cubies)
    {
      for (int i = 0; i < 3; ++i)
      {
        cubie.slot[i] = static_cast<int>(std::lround(cubie.position[i] / this->pitch));
      }
    }
  }

  private: void CreateWelds(EntityComponentManager &_ecm)
  {
    for (auto &cubie : this->cubies)
    {
      const int side = cubie.slot[this->activeAxis];
      Entity parent = this->core;
      if (side != 0)
      {
        parent = this->axles[2 * this->activeAxis + (side > 0 ? 0 : 1)].link;
      }
      cubie.weld = _ecm.CreateEntity();
      _ecm.CreateComponent(
          cubie.weld, components::DetachableJoint({parent, cubie.link, "fixed"}));
      // Not a joint of the model: these two components only make Physics report the wrench
      // the weld transmits.
      _ecm.CreateComponent(cubie.weld, components::Joint());
      _ecm.CreateComponent(cubie.weld, components::JointTransmittedWrench());
    }
  }

  /// Sets the two active layers to exactly the nearest quarter turn, at rest, and the axles
  /// of the axis that is to engage, which carry nothing, to zero.
  private: void Square(EntityComponentManager &_ecm)
  {
    for (int side = 0; side < 2; ++side)
    {
      const Axle &axle = this->axles[2 * this->activeAxis + side];
      const long turns = std::lround(axle.angle / kQuarter);
      _ecm.SetComponentData<components::JointPositionReset>(
          axle.joint, {static_cast<double>(((turns % 4) + 4) % 4) * kQuarter});
      _ecm.SetComponentData<components::JointVelocityReset>(axle.joint, {0.0});
      _ecm.SetComponentData<components::JointPositionReset>(
          this->axles[2 * this->targetAxis + side].joint, {0.0});
    }
  }

  /// Angle of an active layer from the nearest quarter turn.
  private: static double Error(const Axle &_axle)
  {
    return _axle.angle - std::round(_axle.angle / kQuarter) * kQuarter;
  }

  /// Reads from the welds the moment each outer layer needs from the rest of the cube to
  /// stay as it is (`moment`, about the cube axes: minus what the outside applies to it) and
  /// from it the demand on each inactive axis. Returns the inactive axis whose demand is the
  /// largest above the breakaway torque, or -1. Without welds the last moments are kept.
  private: int Demand(const EntityComponentManager &_ecm)
  {
    std::array<std::array<double, 2>, 3> sum{};
    bool read = false;
    for (const auto &cubie : this->cubies)
    {
      auto wrench = _ecm.Component<components::JointTransmittedWrench>(cubie.weld);
      if (!wrench)
      {
        continue;
      }
      read = true;
      const auto &w = wrench->Data();
      // The wrench is expressed in the cubie frame, at the cubie origin.
      const Vector3d force = cubie.rotation.RotateVector(
          Vector3d(w.force().x(), w.force().y(), w.force().z()));
      const Vector3d torque = cubie.position.Cross(force) + cubie.rotation.RotateVector(
          Vector3d(w.torque().x(), w.torque().y(), w.torque().z()));
      for (int axis = 0; axis < 3; ++axis)
      {
        if (cubie.slot[axis] != 0)
        {
          sum[axis][cubie.slot[axis] > 0 ? 0 : 1] += torque[axis];
        }
      }
    }
    if (!read)
    {
      return -1;
    }
    this->moment = sum;
    int best = -1;
    for (int axis = 0; axis < 3; ++axis)
    {
      this->demand[axis] = axis == this->activeAxis ? 0.0 :
          std::max(std::abs(sum[axis][0]), std::abs(sum[axis][1]));
      if (this->demand[axis] > this->breakaway &&
          (best < 0 || this->demand[axis] > this->demand[best]))
      {
        best = axis;
      }
    }
    return best;
  }

  // Parameters (SI units, radians); defaults are overridden by the model's plugin element.
  private: double pitch{0.019};
  private: double breakaway{0.05};
  private: double detentTorque{0.03};
  private: double detentStiffness{3.0};
  private: double detentWindow{0.14};
  private: double snapTolerance{1.75e-3};
  private: double layerInertia{1.5e-5};

  private: bool configured{false};
  private: bool welded{false};
  private: Entity core{kNullEntity};
  private: std::array<Axle, 6> axles;
  private: std::vector<Cubie> cubies;

  private: State state{State::kActive};
  private: int activeAxis{1};
  private: int targetAxis{1};
  private: int settle{0};
  private: int switches{0};
  private: int aborted{0};
  private: int attempts{0};
  private: int asked{0};
  private: int askedAxis{-1};
  private: std::array<double, 2> hold{0.0, 0.0};
  private: std::array<std::array<double, 2>, 3> moment{};
  private: bool blocked{false};
  private: std::array<double, 3> demand{0.0, 0.0, 0.0};

  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher publisher;
};
}  // namespace openfactory

GZ_ADD_PLUGIN(openfactory::RubikCubeSystem, gz::sim::System,
              openfactory::RubikCubeSystem::ISystemConfigure,
              openfactory::RubikCubeSystem::ISystemPreUpdate,
              openfactory::RubikCubeSystem::ISystemPostUpdate)
