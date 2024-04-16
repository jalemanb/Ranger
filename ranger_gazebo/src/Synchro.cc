/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 * Copyright (C) 2023 Benjamin Perseghetti, Rudis Laboratories
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "ranger_gazebo/Synchro.hh"

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/twist.pb.h>

#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Angle.hh>
#include <gz/math/SpeedLimiter.hh>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/common/Console.hh>

#include "gz/sim/components/Actuators.hh"
#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

#include "gz/sim/Model.hh"
#include "gz/sim/components/Pose.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Velocity command.
struct Commands
{
  /// \brief Linear velocity along the x direction.
  double lin_x;

  /// \brief Linear velocity along the y direction.
  double lin_y;

  /// \brief Angular velocity.
  double ang;

  Commands() : lin_x(0.0), lin_y(0.0), ang(0.0) {}
};

class gz::sim::systems::SynchroPrivate
{
  /// \brief Callback for velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel(const msgs::Twist &_msg);

  public: void breakBot(const EntityComponentManager &_ecm);

  public: double controlAngle(double angle);

    public: double velDir(double angle);

  /// \brief Callback for actuator angle subscription
  /// \param[in] _msg angle message
//   public: void OnActuatorAng(const msgs::Actuators &_msg);

  /// \brief Update odometry and publish an odometry message.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateOdometry(const UpdateInfo &_info,
    const EntityComponentManager &_ecm);

  /// \brief Update the linear and angular velocities.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateVelocity(const UpdateInfo &_info,
    const EntityComponentManager &_ecm);

  /// \brief Update the angle.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.


  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Entity of the left joint
  public: std::vector<Entity> tractionJoints;
  public: Entity fl_traction_Joint;
  public: Entity fr_traction_Joint;
  public: Entity rl_traction_Joint;
  public: Entity rr_traction_Joint;

  /// \brief Entity of the left steering joint
  public: std::vector<Entity> steeringJoints;
  public: Entity fl_steering_Joint;
  public: Entity fr_steering_Joint;
  public: Entity rl_steering_Joint;
  public: Entity rr_steering_Joint;

  /// \brief Name of left joint
  public: std::string fl_traction_JointName;
  public: std::string fr_traction_JointName;
  public: std::string rl_traction_JointName;
  public: std::string rr_traction_JointName;

  /// \brief Name of left steering joint
  public: std::string fl_steering_JointName;
  public: std::string fr_steering_JointName;
  public: std::string rl_steering_JointName;
  public: std::string rr_steering_JointName;

  /// \brief Calculated speed of left wheel joint(s)
  public: double fl_traction_JointSpeed{0};
  public: double fr_traction_JointSpeed{0};
  public: double rl_traction_JointSpeed{0};
  public: double rr_traction_JointSpeed{0};

  /// \brief Calculated speed of left joint
  public: double fl_steering_JointSpeed{0};
  public: double fr_steering_JointSpeed{0};
  public: double rl_steering_JointSpeed{0};
  public: double rr_steering_JointSpeed{0};

  /// \brief Distance between front and back wheels
  public: double wheelBase{1.0};

  /// \brief Maximum turning angle to limit steering to
  public: double steeringLimit{0.5};

  /// \brief Wheel radius
  public: double wheelRadius{0.2};

  public: double track{0.0};

  // Odometry and control geometric paramters
  public: double  d{0.0};
  public: double l{0.0};
  public: double a{0.0};

  // Odometry unique parameters
  public: double c_{0.25};
  public: double omega1{0.0};
  public: double omega2{0.0};

  // Odometry Initialized
  public: bool initialized{false};

  // Use grountruth Odometry
  public: bool gt_odom{false};

  /// \brief Allow specifying constant xyz and rpy offsets
  public: gz::math::Pose3d offset = {0, 0, 0, 0, 0, 0};

  /// \brief Index of angle actuator.
  public: int actuatorNumber = 0;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief The model's canonical link.
  public: Link canonicalLink{kNullEntity};

  /// \brief True if using Actuator msg to control steering angle.
  public: bool useActuatorMsg{false};

  /// \brief Update period calculated from <odom__publish_frequency>.
  public: std::chrono::steady_clock::duration odomPubPeriod{0};

  /// \brief Last sim time odom was published.
  public: std::chrono::steady_clock::duration lastOdomPubTime{0};

  /// \brief Ackermann steering odometry message publisher.
  public: transport::Node::Publisher odomPub;

  /// \brief Ackermann tf message publisher.
  public: transport::Node::Publisher tfPub;

  /// \brief Odometry X value
  public: double odomX{0.0};

  /// \brief Odometry Y value
  public: double odomY{0.0};

  /// \brief Odometry yaw value
  public: double odomYaw{0.0};

  /// \brief Odometry old left value
  public: double odomOldLeft{0.0};

  /// \brief Odometry old right value
  public: double odomOldRight{0.0};

  /// \brief Velocity Rolling Window size for Odometer
  public: int velocity_rolling_window_size{3};

  /// \brief Odometry last time value
  public: std::chrono::steady_clock::duration lastOdomTime{0};

  /// \brief Linear velocity limiter.
  public: std::unique_ptr<math::SpeedLimiter> limiterLin;

  /// \brief Angular velocity limiter.
  public: std::unique_ptr<math::SpeedLimiter> limiterAng;

  /// \brief Previous control command.
  public: Commands last0Cmd;

  /// \brief Previous control command to last0Cmd.
  public: Commands last1Cmd;

  /// \brief Last target velocity requested.
  public: msgs::Twist targetVel;

  /// \brief Last target angle requested.
  public: double targetAng{0.0};

  /// \brief P gain for angular position.
  public: double gainPAng{1.0};

  /// \brief P gain for linear velocity.
  public: double gainVLin{1.0};

  /// \brief A mutex to protect the target velocity command.
  public: std::mutex mutex;

  /// \brief frame_id from sdf.
  public: std::string sdfFrameId;

  /// \brief child_frame_id from sdf.
  public: std::string sdfChildFrameId;

  /// \brief Current timestamp.
  public: std::chrono::steady_clock::time_point lastUpdateTime;

  /// \brief Current pose of the model in the odom frame.
  public: math::Pose3d lastUpdatePose{0, 0, 0, 0, 0, 0};

};

//////////////////////////////////////////////////
Synchro::Synchro()
  : dataPtr(std::make_unique<SynchroPrivate>())
{
}

//////////////////////////////////////////////////
void Synchro::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
    this->dataPtr->model = Model(_entity);

    if (!this->dataPtr->model.Valid(_ecm))
    {
        gzerr << "Synchro plugin should be attached to a model entity. "
            << "Failed to initialize." << std::endl;
        return;
    }

    // Get the canonical link
    std::vector<Entity> links = _ecm.ChildrenByComponents(
        this->dataPtr->model.Entity(), components::CanonicalLink());
    if (!links.empty())
        this->dataPtr->canonicalLink = Link(links[0]);


    if (_sdf->HasElement("use_actuator_msg") &&
        _sdf->Get<bool>("use_actuator_msg"))
    {
        if (_sdf->HasElement("actuator_number"))
        {
        this->dataPtr->actuatorNumber =
            _sdf->Get<int>("actuator_number");
        this->dataPtr->useActuatorMsg = true;
        }
        else
        {
        gzerr << "Please specify an actuator_number" <<
            "to use Actuator position message control." << std::endl;
        }
    }

    // Get params from SDF (Steering joints)
    auto sdfFlSteerElem = _sdf->FindElement("fl_steering_joint");
    this->dataPtr->fl_steering_JointName = sdfFlSteerElem->Get<std::string>();
    auto sdfFrSteerElem = _sdf->FindElement("fr_steering_joint");
    this->dataPtr->fr_steering_JointName = sdfFrSteerElem->Get<std::string>();
    auto sdfRlSteerElem = _sdf->FindElement("rl_steering_joint");
    this->dataPtr->rl_steering_JointName = sdfRlSteerElem->Get<std::string>();
    auto sdfRrSteerElem = _sdf->FindElement("rr_steering_joint");
    this->dataPtr->rr_steering_JointName = sdfRrSteerElem->Get<std::string>();

    // Get params from SDF (Traction joints)
    auto sdfFlTractionElem = _sdf->FindElement("fl_traction_joint");
    this->dataPtr->fl_traction_JointName = sdfFlTractionElem->Get<std::string>();
    auto sdfFrTractionElem = _sdf->FindElement("fr_traction_joint");
    this->dataPtr->fr_traction_JointName = sdfFrTractionElem->Get<std::string>();
    auto sdfRlTractionElem = _sdf->FindElement("rl_traction_joint");
    this->dataPtr->rl_traction_JointName = sdfRlTractionElem->Get<std::string>();
    auto sdfRrTractionElem = _sdf->FindElement("rr_traction_joint");
    this->dataPtr->rr_traction_JointName = sdfRrTractionElem->Get<std::string>();

    // Get params from SDF (Kinematics)
    this->dataPtr->wheelRadius = _sdf->Get<double>("wheel_radius",
      this->dataPtr->wheelRadius).first;

    this->dataPtr->wheelBase = _sdf->Get<double>("wheel_base",
        this->dataPtr->wheelBase).first;

    this->dataPtr->steeringLimit = _sdf->Get<double>("steering_limit",
        this->dataPtr->steeringLimit).first;

    this->dataPtr->track = _sdf->Get<double>("track",
        this->dataPtr->track).first;

    // Get params from SDF (Control)
    this->dataPtr->gainPAng = _sdf->Get<double>("p_gain_steering",
        this->dataPtr->gainPAng).first;

    this->dataPtr->gainVLin = _sdf->Get<double>("p_gain_vel",
        this->dataPtr->gainVLin).first;

    // Use ground Truth Odometry
    this->dataPtr->gt_odom = _sdf->Get<bool>("gt_odom",
        this->dataPtr->gt_odom).first;

    // Setup Control/Odometry Parameters
    this->dataPtr->d = this->dataPtr->track/2;
    this->dataPtr->l = this->dataPtr->wheelBase/2;
    this->dataPtr->a = this->dataPtr->wheelRadius;

    // Setup Odometry Unique Parameters
    this->dataPtr->omega1 = this->dataPtr->l / ((4*this->dataPtr->d*this->dataPtr->d) + (4*this->dataPtr->l*this->dataPtr->l));

    this->dataPtr->omega2 = this->dataPtr->d / ((4*this->dataPtr->d*this->dataPtr->d) + (4*this->dataPtr->l*this->dataPtr->l));

    // Instantiate the speed limiters.
    this->dataPtr->limiterLin = std::make_unique<math::SpeedLimiter>();
    this->dataPtr->limiterAng = std::make_unique<math::SpeedLimiter>();

    // Offset Management for RViz Odometry Visualization
    if (_sdf->HasElement("xyz_offset"))
    {
        this->dataPtr->offset.Pos() = _sdf->Get<gz::math::Vector3d>(
        "xyz_offset");
    }

    if (_sdf->HasElement("rpy_offset"))
    {
        this->dataPtr->offset.Rot() =
        gz::math::Quaterniond(_sdf->Get<gz::math::Vector3d>(
            "rpy_offset"));
    }

    // Parse speed limiter parameters.
    if (_sdf->HasElement("min_velocity"))
    {
        const double minVel = _sdf->Get<double>("min_velocity");
        this->dataPtr->limiterLin->SetMinVelocity(minVel);
        this->dataPtr->limiterAng->SetMinVelocity(minVel);
    }
    if (_sdf->HasElement("max_velocity"))
    {
        const double maxVel = _sdf->Get<double>("max_velocity");
        this->dataPtr->limiterLin->SetMaxVelocity(maxVel);
        this->dataPtr->limiterAng->SetMaxVelocity(maxVel);
    }
    if (_sdf->HasElement("min_acceleration"))
    {
        const double minAccel = _sdf->Get<double>("min_acceleration");
        this->dataPtr->limiterLin->SetMinAcceleration(minAccel);
        this->dataPtr->limiterAng->SetMinAcceleration(minAccel);
    }
    if (_sdf->HasElement("max_acceleration"))
    {
        const double maxAccel = _sdf->Get<double>("max_acceleration");
        this->dataPtr->limiterLin->SetMaxAcceleration(maxAccel);
        this->dataPtr->limiterAng->SetMaxAcceleration(maxAccel);
    }
    if (_sdf->HasElement("min_jerk"))
    {
        const double minJerk = _sdf->Get<double>("min_jerk");
        this->dataPtr->limiterLin->SetMinJerk(minJerk);
        this->dataPtr->limiterAng->SetMinJerk(minJerk);
    }
    if (_sdf->HasElement("max_jerk"))
    {
        const double maxJerk = _sdf->Get<double>("max_jerk");
        this->dataPtr->limiterLin->SetMaxJerk(maxJerk);
        this->dataPtr->limiterAng->SetMaxJerk(maxJerk);
    }

    double odomFreq = _sdf->Get<double>("odom_publish_frequency", 50).first;
    if (odomFreq > 0)
    {
      std::chrono::duration<double> odomPer{1 / odomFreq};
      this->dataPtr->odomPubPeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          odomPer);
    }
  
    // Subscribe to commands
    std::vector<std::string> topics;

    topics.push_back("/model/" + this->dataPtr->model.Name(_ecm) + "/cmd_vel");

    auto topic = validTopic(topics);
    if (topic.empty())
    {
        gzerr << "Synchro plugin received invalid topic name "
            << "Failed to initialize." << std::endl;
        return;
    }

    this->dataPtr->node.Subscribe(topic, &SynchroPrivate::OnCmdVel, this->dataPtr.get());
    gzmsg << "Synchro subscribing to twist messages on ["
          << topic << "]" << std::endl;
  
    std::vector<std::string> odomTopics;
    if (_sdf->HasElement("odom_topic"))
    {
      odomTopics.push_back(_sdf->Get<std::string>("odom_topic"));
    }
    odomTopics.push_back("/model/" + this->dataPtr->model.Name(_ecm) +
        "/odometry");
    auto odomTopic = validTopic(odomTopics);
    if (topic.empty())
    {
      gzerr << "Synchro plugin received invalid model name "
            << "Failed to initialize." << std::endl;
      return;
    }

    this->dataPtr->odomPub = this->dataPtr->node.Advertise<msgs::Odometry>(
        odomTopic);

    std::vector<std::string> tfTopics;
    if (_sdf->HasElement("tf_topic"))
    {
      tfTopics.push_back(_sdf->Get<std::string>("tf_topic"));
    }
    tfTopics.push_back("/model/" + this->dataPtr->model.Name(_ecm) +
      "/tf");
    auto tfTopic = validTopic(tfTopics);
    if (tfTopic.empty())
    {
      gzerr << "Synchro plugin invalid tf topic name "
            << "Failed to initialize." << std::endl;
      return;
    }

    this->dataPtr->tfPub = this->dataPtr->node.Advertise<msgs::Pose_V>(
        tfTopic);

    if (_sdf->HasElement("frame_id"))
      this->dataPtr->sdfFrameId = _sdf->Get<std::string>("frame_id");

    if (_sdf->HasElement("child_frame_id"))
      this->dataPtr->sdfChildFrameId = _sdf->Get<std::string>("child_frame_id");
}

//////////////////////////////////////////////////
void Synchro::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("Synchro::PreUpdate");

    // \TODO(anyone) Support rewind
    if (_info.dt < std::chrono::steady_clock::duration::zero())
    {
        gzwarn << "Detected jump back in time ["
            << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
            << "s]. System may not work properly." << std::endl;
    }

    // If the joints haven't been identified yet, look for them
    static std::set<std::string> warnedModels;
    auto modelName = this->dataPtr->model.Name(_ecm);
    if (this->dataPtr->tractionJoints.empty())
    {
        bool warned{false};

        Entity joint_fl = this->dataPtr->model.JointByName(_ecm, this->dataPtr->fl_traction_JointName);
        if (joint_fl != kNullEntity)
        {
            this->dataPtr->fl_traction_Joint = joint_fl;
            this->dataPtr->tractionJoints.push_back(joint_fl);
        }
        else if (warnedModels.find(modelName) == warnedModels.end())
        {
            gzwarn << "Failed to find traction joint [" << this->dataPtr->fl_traction_JointName << "] for model ["
                    << modelName << "]" << std::endl;
            warned = true;
        }
        Entity joint_fr = this->dataPtr->model.JointByName(_ecm, this->dataPtr->fr_traction_JointName);
        if (joint_fr != kNullEntity)
        {
            this->dataPtr->fr_traction_Joint = joint_fr;
            this->dataPtr->tractionJoints.push_back(joint_fr);
        }
        else if (warnedModels.find(modelName) == warnedModels.end())
        {
            gzwarn << "Failed to find traction joint [" << this->dataPtr->fr_traction_JointName << "] for model ["
                    << modelName << "]" << std::endl;
            warned = true;
        }
        Entity joint_rl = this->dataPtr->model.JointByName(_ecm, this->dataPtr->rl_traction_JointName);
        if (joint_rl != kNullEntity)
        {
            this->dataPtr->rl_traction_Joint = joint_rl;
            this->dataPtr->tractionJoints.push_back(joint_rl);
        }
        else if (warnedModels.find(modelName) == warnedModels.end())
        {
            gzwarn << "Failed to find traction joint [" << this->dataPtr->rl_traction_JointName << "] for model ["
                    << modelName << "]" << std::endl;
            warned = true;
        }
        Entity joint_rr = this->dataPtr->model.JointByName(_ecm, this->dataPtr->rr_traction_JointName);
        if (joint_rr != kNullEntity)
        {
            this->dataPtr->rr_traction_Joint = joint_rr;
            this->dataPtr->tractionJoints.push_back(joint_rr);
        }
        else if (warnedModels.find(modelName) == warnedModels.end())
        {
            gzwarn << "Failed to find traction joint [" << this->dataPtr->rr_traction_JointName << "] for model ["
                    << modelName << "]" << std::endl;
            warned = true;
        }
        if (warned)
        {
        warnedModels.insert(modelName);
        }
    }

    if (this->dataPtr->steeringJoints.empty())
    {
        bool warned{false};

        Entity joint_fl = this->dataPtr->model.JointByName(_ecm, this->dataPtr->fl_steering_JointName);
        if (joint_fl != kNullEntity)
        {
            this->dataPtr->fl_steering_Joint = joint_fl;
            this->dataPtr->steeringJoints.push_back(joint_fl);
        }
        else if (warnedModels.find(modelName) == warnedModels.end())
        {
            gzwarn << "Failed to find traction joint [" << this->dataPtr->fl_steering_JointName << "] for model ["
                    << modelName << "]" << std::endl;
            warned = true;
        }
        Entity joint_fr = this->dataPtr->model.JointByName(_ecm, this->dataPtr->fr_steering_JointName);
        if (joint_fr != kNullEntity)
        {
            this->dataPtr->fr_steering_Joint = joint_fr;
            this->dataPtr->steeringJoints.push_back(joint_fr);
        }
        else if (warnedModels.find(modelName) == warnedModels.end())
        {
            gzwarn << "Failed to find traction joint [" << this->dataPtr->fr_steering_JointName << "] for model ["
                    << modelName << "]" << std::endl;
            warned = true;
        }
        Entity joint_rl = this->dataPtr->model.JointByName(_ecm, this->dataPtr->rl_steering_JointName);
        if (joint_rl != kNullEntity)
        {
            this->dataPtr->rl_steering_Joint = joint_rl;
            this->dataPtr->steeringJoints.push_back(joint_rl);
        }
        else if (warnedModels.find(modelName) == warnedModels.end())
        {
            gzwarn << "Failed to find traction joint [" << this->dataPtr->rl_steering_JointName << "] for model ["
                    << modelName << "]" << std::endl;
            warned = true;
        }
        Entity joint_rr = this->dataPtr->model.JointByName(_ecm, this->dataPtr->rr_steering_JointName);
        if (joint_rr != kNullEntity)
        {
            this->dataPtr->rr_steering_Joint = joint_rr;
            this->dataPtr->steeringJoints.push_back(joint_rr);
        }
        else if (warnedModels.find(modelName) == warnedModels.end())
        {
            gzwarn << "Failed to find traction joint [" << this->dataPtr->rr_steering_JointName << "] for model ["
                    << modelName << "]" << std::endl;
            warned = true;
        }
        if (warned)
        {
        warnedModels.insert(modelName);
        }
    }

    if (this->dataPtr->tractionJoints.empty())
        return;
    else if (this->dataPtr->steeringJoints.empty())
        return;

    if (warnedModels.find(modelName) != warnedModels.end())
    {
        gzmsg << "Found joints for model [" << modelName
            << "], plugin will start working." << std::endl;
        warnedModels.erase(modelName);
    }

    // Nothing left to do if paused.
    if (_info.paused)
        return;

    // Update traction velocities
    auto fl_trac_vel = _ecm.Component<components::JointVelocityCmd>(this->dataPtr->fl_traction_Joint);
    if (fl_trac_vel == nullptr)
    {
    _ecm.CreateComponent(
        this->dataPtr->fl_traction_Joint, components::JointVelocityCmd(
            {this->dataPtr->fl_traction_JointSpeed}));
    }
    else
    {
        *fl_trac_vel = components::JointVelocityCmd({this->dataPtr->fl_traction_JointSpeed});
    }
    auto fr_trac_vel = _ecm.Component<components::JointVelocityCmd>(this->dataPtr->fr_traction_Joint);
    if (fr_trac_vel == nullptr)
    {
    _ecm.CreateComponent(
        this->dataPtr->fr_traction_Joint, components::JointVelocityCmd(
            {this->dataPtr->fr_traction_JointSpeed}));
    }
    else
    {
        *fr_trac_vel = components::JointVelocityCmd({this->dataPtr->fr_traction_JointSpeed});
    }
    auto rl_trac_vel = _ecm.Component<components::JointVelocityCmd>(this->dataPtr->rl_traction_Joint);
    if (rl_trac_vel == nullptr)
    {
    _ecm.CreateComponent(
        this->dataPtr->rl_traction_Joint, components::JointVelocityCmd(
            {this->dataPtr->rl_traction_JointSpeed}));
    }
    else
    {
        *rl_trac_vel = components::JointVelocityCmd({this->dataPtr->rl_traction_JointSpeed});
    }
    auto rr_trac_vel = _ecm.Component<components::JointVelocityCmd>(this->dataPtr->rr_traction_Joint);
    if (rr_trac_vel == nullptr)
    {
    _ecm.CreateComponent(
        this->dataPtr->rr_traction_Joint, components::JointVelocityCmd(
            {this->dataPtr->rr_traction_JointSpeed}));
    }
    else
    {
        *rr_trac_vel = components::JointVelocityCmd({this->dataPtr->rr_traction_JointSpeed});
    }
    // Update steering velocities
    auto fl_steer_vel = _ecm.Component<components::JointVelocityCmd>(this->dataPtr->fl_steering_Joint);
    if (fl_steer_vel == nullptr)
    {
    _ecm.CreateComponent(
        this->dataPtr->fl_steering_Joint, components::JointVelocityCmd(
            {this->dataPtr->fl_steering_JointSpeed}));
    }
    else
    {
        *fl_steer_vel = components::JointVelocityCmd({this->dataPtr->fl_steering_JointSpeed});
    }
    auto fr_steer_vel = _ecm.Component<components::JointVelocityCmd>(this->dataPtr->fr_steering_Joint);
    if (fr_steer_vel == nullptr)
    {
    _ecm.CreateComponent(
        this->dataPtr->fr_steering_Joint, components::JointVelocityCmd(
            {this->dataPtr->fr_steering_JointSpeed}));
    }
    else
    {
        *fr_steer_vel = components::JointVelocityCmd({this->dataPtr->fr_steering_JointSpeed});
    }
    auto rl_steer_vel = _ecm.Component<components::JointVelocityCmd>(this->dataPtr->rl_steering_Joint);
    if (rl_steer_vel == nullptr)
    {
    _ecm.CreateComponent(
        this->dataPtr->rl_steering_Joint, components::JointVelocityCmd(
            {this->dataPtr->rl_steering_JointSpeed}));
    }
    else
    {
        *rl_steer_vel = components::JointVelocityCmd({this->dataPtr->rl_steering_JointSpeed});
    }
    auto rr_steer_vel = _ecm.Component<components::JointVelocityCmd>(this->dataPtr->rr_steering_Joint);
    if (rr_steer_vel == nullptr)
    {
    _ecm.CreateComponent(
        this->dataPtr->rr_steering_Joint, components::JointVelocityCmd(
            {this->dataPtr->rr_steering_JointSpeed}));
    }
    else
    {
        *rr_steer_vel = components::JointVelocityCmd({this->dataPtr->rr_steering_JointSpeed});
    }
    // Update traction posititions
    auto fl_trac_pos = _ecm.Component<components::JointPosition>(
        this->dataPtr->fl_traction_Joint);
    if (!fl_trac_pos)
    {
        _ecm.CreateComponent(this->dataPtr->fl_traction_Joint,
            components::JointPosition());
    }
    auto fr_trac_pos = _ecm.Component<components::JointPosition>(
        this->dataPtr->fr_traction_Joint);
    if (!fr_trac_pos)
    {
        _ecm.CreateComponent(this->dataPtr->fr_traction_Joint,
            components::JointPosition());
    }
    auto rl_trac_pos = _ecm.Component<components::JointPosition>(
        this->dataPtr->rl_traction_Joint);
    if (!rl_trac_pos)
    {
    _ecm.CreateComponent(this->dataPtr->rl_traction_Joint,
        components::JointPosition());
    }
    auto rr_trac_pos = _ecm.Component<components::JointPosition>(
        this->dataPtr->rr_traction_Joint);
    if (!rr_trac_pos)
    {
    _ecm.CreateComponent(this->dataPtr->rr_traction_Joint,
        components::JointPosition());
    }
    // Update Steering posititions
    auto fl_steer_pos = _ecm.Component<components::JointPosition>(
        this->dataPtr->fl_steering_Joint);
    if (!fl_steer_pos)
    {
        _ecm.CreateComponent(this->dataPtr->fl_steering_Joint,
            components::JointPosition());
    }
    auto fr_steer_pos = _ecm.Component<components::JointPosition>(
        this->dataPtr->fr_steering_Joint);
    if (!fr_steer_pos)
    {
        _ecm.CreateComponent(this->dataPtr->fr_steering_Joint,
            components::JointPosition());
    }
    auto rl_steer_pos = _ecm.Component<components::JointPosition>(
        this->dataPtr->rl_steering_Joint);
    if (!rl_steer_pos)
    {
    _ecm.CreateComponent(this->dataPtr->rl_steering_Joint,
        components::JointPosition());
    }
    auto rr_steer_pos = _ecm.Component<components::JointPosition>(
        this->dataPtr->rr_steering_Joint);
    if (!rr_steer_pos)
    {
    _ecm.CreateComponent(this->dataPtr->rr_steering_Joint,
        components::JointPosition());
    }
}

//////////////////////////////////////////////////
void Synchro::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
    GZ_PROFILE("Synchro::PostUpdate");
    // Nothing left to do if paused.
    if (_info.paused)
    {
        return;
    }

    this->dataPtr->UpdateVelocity(_info, _ecm);
    this->dataPtr->UpdateOdometry(_info, _ecm);
}

//////////////////////////////////////////////////
void SynchroPrivate::UpdateOdometry(
    const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
    GZ_PROFILE("Synchro::UpdateOdometry");
    // Initialize, if not already initialized.
    if (!this->initialized)
    {
        this->lastUpdateTime = std::chrono::steady_clock::time_point(_info.simTime);
        this->initialized = true;
        return;
    }

    double linvx{0.0}, linvy{0.0}, angw{0.0};
    math::Pose3d pose_base_link;


    if (!gt_odom)
    {

        if (this->tractionJoints.empty() ||
            this->steeringJoints.empty() )
            return;

        // get the position of all joints
        auto fl_trac_pos = _ecm.Component<components::JointPosition>(this->fl_traction_Joint); double fl_trac_pos_ = fl_trac_pos->Data()[0];
        auto fr_trac_pos = _ecm.Component<components::JointPosition>(this->fr_traction_Joint); double fr_trac_pos_ = fr_trac_pos->Data()[0];
        auto rl_trac_pos = _ecm.Component<components::JointPosition>(this->rl_traction_Joint); double rl_trac_pos_ = rl_trac_pos->Data()[0];
        auto rr_trac_pos = _ecm.Component<components::JointPosition>(this->rr_traction_Joint); double rr_trac_pos_ = rr_trac_pos->Data()[0];

        auto fl_steer_pos = _ecm.Component<components::JointPosition>(this->fl_steering_Joint); double fl_steer_pos_ = fl_steer_pos->Data()[0];
        auto fr_steer_pos = _ecm.Component<components::JointPosition>(this->fr_steering_Joint); double fr_steer_pos_ = fr_steer_pos->Data()[0];
        auto rl_steer_pos = _ecm.Component<components::JointPosition>(this->rl_steering_Joint); double rl_steer_pos_ = rl_steer_pos->Data()[0];
        auto rr_steer_pos = _ecm.Component<components::JointPosition>(this->rr_steering_Joint); double rr_steer_pos_ = rr_steer_pos->Data()[0];

        // get the velocities of all joints
        auto fl_trac_vel = _ecm.Component<components::JointVelocity>(this->fl_traction_Joint); double fl_trac_vel_ = fl_trac_vel->Data()[0];
        auto fr_trac_vel = _ecm.Component<components::JointVelocity>(this->fr_traction_Joint); double fr_trac_vel_ = fr_trac_vel->Data()[0];
        auto rl_trac_vel = _ecm.Component<components::JointVelocity>(this->rl_traction_Joint); double rl_trac_vel_ = rl_trac_vel->Data()[0];
        auto rr_trac_vel = _ecm.Component<components::JointVelocity>(this->rr_traction_Joint); double rr_trac_vel_ = rr_trac_vel->Data()[0];

        auto fl_steer_vel = _ecm.Component<components::JointVelocity>(this->fl_steering_Joint); double fl_steer_vel_ = fl_steer_vel->Data()[0];
        auto fr_steer_vel = _ecm.Component<components::JointVelocity>(this->fr_steering_Joint); double fr_steer_vel_ = fr_steer_vel->Data()[0];
        auto rl_steer_vel = _ecm.Component<components::JointVelocity>(this->rl_steering_Joint); double rl_steer_vel_ = rl_steer_vel->Data()[0];
        auto rr_steer_vel = _ecm.Component<components::JointVelocity>(this->rr_steering_Joint); double rr_steer_vel_ = rr_steer_vel->Data()[0];

        // Abort if the joints were not found or just created.
        if (!fl_trac_pos  || fl_trac_vel->Data().empty()  || 
            !fr_trac_pos  || fr_trac_vel->Data().empty()  || 
            !rl_trac_pos  || rl_trac_vel->Data().empty()  ||
            !rr_trac_pos  || rr_trac_vel->Data().empty()  ||
            !fl_steer_pos || fl_steer_vel->Data().empty() ||
            !fr_steer_pos || fr_steer_vel->Data().empty() ||
            !rl_steer_pos || rl_steer_vel->Data().empty() ||
            !rr_steer_pos || rr_steer_vel->Data().empty() || 
            !fl_trac_vel  || fl_trac_vel->Data().empty()  || 
            !fr_trac_vel  || fr_trac_vel->Data().empty()  || 
            !rl_trac_vel  || rl_trac_vel->Data().empty()  ||
            !rr_trac_vel  || rr_trac_vel->Data().empty()  ||
            !fl_steer_vel || fl_steer_vel->Data().empty() ||
            !fr_steer_vel || fr_steer_vel->Data().empty() ||
            !rl_steer_vel || rl_steer_vel->Data().empty() ||
            !rr_steer_vel || rr_steer_vel->Data().empty())
        {
            return;
        }

        if (std::isnan(fl_trac_vel_) || std::isnan(fr_trac_vel_) || std::isnan(rl_trac_vel_) || std::isnan(rr_trac_vel_))
            {return;}
        
        if (std::isnan(fl_steer_pos_) || std::isnan(fr_steer_pos_) || std::isnan(rl_steer_pos_) || std::isnan(rr_steer_pos_))
            {return;}

        // Calculate the odometry

        // Find the required wheel linear speed in the respective x and y direction
        double v1x{0.0}, v1y{0.0},
            v2x{0.0}, v2y{0.0}, 
            v3x{0.0}, v3y{0.0}, 
            v4x{0.0}, v4y{0.0};

        double vx, vy, w;

        v1x = a*fl_trac_vel_*std::cos(fl_steer_pos_);
        v1y = a*fl_trac_vel_*std::sin(fl_steer_pos_);

        v2x = a*rl_trac_vel_*std::cos(rl_steer_pos_);
        v2y = a*rl_trac_vel_*std::sin(rl_steer_pos_);

        v3x = a*rr_trac_vel_*std::cos(rr_steer_pos_);
        v3y = a*rr_trac_vel_*std::sin(rr_steer_pos_);
        
        v4x = a*fr_trac_vel_*std::cos(fr_steer_pos_);
        v4y = a*fr_trac_vel_*std::sin(fr_steer_pos_);

        vx = c_*v1x + c_*v2x + c_*v3x + c_*v4x;
        vy = c_*v1y + c_*v2y + c_*v3y + c_*v4y;

        w = - omega2*v1x + omega1*v1y - omega2*v2x - omega1*v2y + omega2*v3x - omega1*v3y + omega2*v4x + omega1*v4y;

        double dt_secs = _info.dt.count()/1e9;

        // Integration
        this->odomYaw += dt_secs*w;
        this->odomYaw = math::Angle(this->odomYaw).Normalized().Radian();
        this->odomX += dt_secs*vx*std::cos(this->odomYaw) - dt_secs*vy*std::sin(this->odomYaw);
        this->odomY += dt_secs*vx*std::sin(this->odomYaw) + dt_secs*vy*std::cos(this->odomYaw);

        linvx = vx;
        linvy = vy;
        angw = w;
        // gzmsg << "Using Real Odom: "  << std::endl;

    }
    else
    {
        const std::chrono::duration<double> dt = std::chrono::steady_clock::time_point(_info.simTime) - this->lastUpdateTime;

        if (math::equal(0.0, dt.count())){return;}

        pose_base_link = worldPose(this->model.Entity(), _ecm);

        // Get linear and angular displacements from last updated pose.
        double linearDisplacementX = pose_base_link.Pos().X() - this->lastUpdatePose.Pos().X();
        double linearDisplacementY = pose_base_link.Pos().Y() - this->lastUpdatePose.Pos().Y();
        double currentYaw = pose_base_link.Rot().Yaw();
        const double lastYaw = this->lastUpdatePose.Rot().Yaw();
        while (currentYaw < lastYaw - M_PI) currentYaw += 2 * M_PI;
        while (currentYaw > lastYaw + M_PI) currentYaw -= 2 * M_PI;
        const float yawDiff = currentYaw - lastYaw;

        this->odomX = pose_base_link.Pos().X();
        this->odomY = pose_base_link.Pos().Y();
        this->odomYaw = pose_base_link.Rot().Yaw();

        linvx = linearDisplacementX / dt.count();
        linvy = linearDisplacementY / dt.count();
        angw = yawDiff / dt.count();

        this->lastUpdatePose = pose_base_link;
        this->lastUpdateTime = std::chrono::steady_clock::time_point(_info.simTime);
    }

  // Throttle odometry publishing
  auto diff = _info.simTime - this->lastOdomPubTime;
  if (diff > std::chrono::steady_clock::duration::zero() &&
      diff < this->odomPubPeriod)
  {
    return;
  }
  this->lastOdomPubTime = _info.simTime;

  // Construct the odometry message and publish it.
  msgs::Odometry msg;
  msg.mutable_pose()->mutable_position()->set_x(this->odomX);
  msg.mutable_pose()->mutable_position()->set_y(this->odomY);

  if (gt_odom)
  {
    msg.mutable_pose()->mutable_position()->set_z(pose_base_link.Pos().Z());
  }
  else 
  {
    msg.mutable_pose()->mutable_position()->set_z(this->offset.Pos().Z());
  }

  math::Quaterniond orientation(0, 0, this->odomYaw);
  msgs::Set(msg.mutable_pose()->mutable_orientation(), orientation);

  msg.mutable_twist()->mutable_linear()->set_x(linvx);
  msg.mutable_twist()->mutable_linear()->set_y(linvy);
  msg.mutable_twist()->mutable_angular()->set_z(angw);

  // Set the time stamp in the header
  msg.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));

  // Set the frame id.
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  if (this->sdfFrameId.empty())
  {
    frame->add_value(this->model.Name(_ecm) + "/odom");
  }
  else
  {
    frame->add_value(this->sdfFrameId);
  }

  std::optional<std::string> linkName = this->canonicalLink.Name(_ecm);
  if (this->sdfChildFrameId.empty())
  {
    if (linkName)
    {
      auto childFrame = msg.mutable_header()->add_data();
      childFrame->set_key("child_frame_id");
      childFrame->add_value(this->model.Name(_ecm) + "/" + *linkName);
    }
  }
  else
  {
    auto childFrame = msg.mutable_header()->add_data();
    childFrame->set_key("child_frame_id");
    childFrame->add_value(this->sdfChildFrameId);
  }

  // Construct the Pose_V/tf message and publish it.
  msgs::Pose_V tfMsg;
  msgs::Pose *tfMsgPose = tfMsg.add_pose();
  tfMsgPose->mutable_header()->CopyFrom(*msg.mutable_header());
  tfMsgPose->mutable_position()->CopyFrom(msg.mutable_pose()->position());
  tfMsgPose->mutable_orientation()->CopyFrom(msg.mutable_pose()->orientation());

  // Publish the message
  this->odomPub.Publish(msg);
  this->tfPub.Publish(tfMsg);
}

////////////////////////////////////////////////
void SynchroPrivate::UpdateVelocity(
    const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
    GZ_PROFILE("Synchro::UpdateVelocity");


    double linVel_x;
    double linVel_y;
    double angVel;
    {
        std::lock_guard<std::mutex> lock(this->mutex);
        linVel_x = this->targetVel.linear().x();
        linVel_y = this->targetVel.linear().y();
        angVel = this->targetVel.angular().z();
    }


    double front_left_steering = 0, front_right_steering = 0;
    double rear_left_steering = 0, rear_right_steering = 0;

    // Limit the target velocity if needed.
    this->limiterLin->Limit(
        linVel_x, this->last0Cmd.lin_x, this->last1Cmd.lin_x, _info.dt);
    this->limiterLin->Limit(
        linVel_y, this->last0Cmd.lin_y, this->last1Cmd.lin_y, _info.dt);
    this->limiterAng->Limit(
        angVel, this->last0Cmd.ang, this->last1Cmd.ang, _info.dt);

    // Update history of commands.
    this->last1Cmd = last0Cmd;
    this->last0Cmd.lin_x = linVel_x;
    this->last0Cmd.lin_y = linVel_y;
    this->last0Cmd.ang = angVel;

    if (std::hypot(linVel_x, linVel_y, angVel) < 0.001)
    {
        this->breakBot(_ecm);
    }
    else
    {
        // Find the required wheel linear speed in the respective x and y direction
        double v1x{0.0}, v1y{0.0}, theta1{0.0},
               v2x{0.0}, v2y{0.0}, theta2{0.0},
               v3x{0.0}, v3y{0.0}, theta3{0.0},
               v4x{0.0}, v4y{0.0}, theta4{0.0};

        double d, l, a;
        d = track/2;
        l = wheelBase/2;
        a = wheelRadius;

        v1x = linVel_x - (d*angVel);
        v1y = linVel_y + (l*angVel);

        v2x = linVel_x - (d*angVel);
        v2y = linVel_y - (l*angVel);

        v3x = linVel_x + (d*angVel);
        v3y = linVel_y - (l*angVel);

        v4x = linVel_x + (d*angVel);
        v4y = linVel_y + (l*angVel);

        theta1 = std::atan2(v1y, v1x);
        theta2 = std::atan2(v2y, v2x);
        theta3 = std::atan2(v3y, v3x);
        theta4 =  std::atan2(v4y, v4x);

        // Compute joint velocities
        this->fl_traction_JointSpeed = this->gainVLin*velDir(theta1)*(1/a)*std::hypot(v1x, v1y);
        this->fr_traction_JointSpeed = this->gainVLin*velDir(theta4)*(1/a)*std::hypot(v4x, v4y);
        this->rl_traction_JointSpeed = this->gainVLin*velDir(theta2)*(1/a)*std::hypot(v2x, v2y);
        this->rr_traction_JointSpeed = this->gainVLin*velDir(theta3)*(1/a)*std::hypot(v3x, v3y);

        // Compute Steering angles
        front_left_steering = controlAngle(theta1);
        front_right_steering = controlAngle(theta4);
        rear_left_steering = controlAngle(theta2);
        rear_right_steering = controlAngle(theta3);

        // Compute Steering velocities
        auto fl_steer_pos = _ecm.Component<components::JointPosition>(this->fl_steering_Joint); 
        auto fr_steer_pos = _ecm.Component<components::JointPosition>(this->fr_steering_Joint); 
        auto rl_steer_pos = _ecm.Component<components::JointPosition>(this->rl_steering_Joint); 
        auto rr_steer_pos = _ecm.Component<components::JointPosition>(this->rr_steering_Joint); 

        // Abort if the joints were not found or just created.
        if (!fl_steer_pos || !fr_steer_pos || !rl_steer_pos || !rr_steer_pos ||
            fl_steer_pos->Data().empty() || fr_steer_pos->Data().empty() ||  rl_steer_pos->Data().empty() || rr_steer_pos->Data().empty())
        {
            return;
        }
        
        // Find the desired joint speed (Approximation not considering dt) 
        double flDelta = front_left_steering - fl_steer_pos->Data()[0];
        double frDelta = front_right_steering - fr_steer_pos->Data()[0];
        double rlDelta = rear_left_steering - rl_steer_pos->Data()[0];
        double rrDelta = rear_right_steering - rr_steer_pos->Data()[0];

        // Setting Joint Velocity to steer each wheel
        this->fl_steering_JointSpeed = this->gainPAng*flDelta;
        this->fr_steering_JointSpeed = this->gainPAng*frDelta;
        this->rl_steering_JointSpeed = this->gainPAng*rlDelta;
        this->rr_steering_JointSpeed = this->gainPAng*rrDelta;
    }
}

//////////////////////////////////////////////////
void SynchroPrivate::OnCmdVel(const msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->targetVel = _msg;
}

//////////////////////////////////////////////////
void SynchroPrivate::breakBot(const EntityComponentManager &_ecm)
{
    this->fl_traction_JointSpeed =
    this->fr_traction_JointSpeed =
    this->rl_traction_JointSpeed =
    this->rr_traction_JointSpeed = 0.0;

    // Compute Steering velocities
    auto fl_steer_pos = _ecm.Component<components::JointPosition>(this->fl_steering_Joint); 
    auto fr_steer_pos = _ecm.Component<components::JointPosition>(this->fr_steering_Joint); 
    auto rl_steer_pos = _ecm.Component<components::JointPosition>(this->rl_steering_Joint); 
    auto rr_steer_pos = _ecm.Component<components::JointPosition>(this->rr_steering_Joint); 

    double flDelta = 0.0 - fl_steer_pos->Data()[0];
    double frDelta = 0.0 - fr_steer_pos->Data()[0];
    double rlDelta = 0.0 - rl_steer_pos->Data()[0];
    double rrDelta = 0.0 - rr_steer_pos->Data()[0];

    // Setting Joint Velocity to steer each wheel
    this->fl_steering_JointSpeed = this->gainPAng*flDelta;
    this->fr_steering_JointSpeed = this->gainPAng*frDelta;
    this->rl_steering_JointSpeed = this->gainPAng*rlDelta;
    this->rr_steering_JointSpeed = this->gainPAng*rrDelta;
}

double SynchroPrivate::controlAngle(double angle)
{
    if (angle > M_PI/2)
    {
        return angle - M_PI;
    }
    else if (angle < -M_PI/2)
    {
        return angle + M_PI;
    }
    else
    {
        return angle;
    }
    
}

double SynchroPrivate::velDir(double angle)
{
    if (angle > M_PI/2 || angle < -M_PI/2)
    {
        return -1;
    }
    else
    {
        return 1;
    }
    
}

//////////////////////////////////////////////////

GZ_ADD_PLUGIN(Synchro,
              System,
              Synchro::ISystemConfigure,
              Synchro::ISystemPreUpdate,
              Synchro::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(Synchro,
                    "gz::sim::systems::Synchro")