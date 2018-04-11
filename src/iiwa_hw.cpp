/*
* This class implements a bridge between ROS hardware interfaces and a KUKA LBR IIWA Robot,
* using KUKA Sunrise.FRI and iiwa_stack repo.
*
* It combines multiple works from
* https://github.com/RobotLocomotion/drake-iiwa-driver.git
* https://github.com/SalvoVirga/iiwa_stack.git
* https://github.com/ahundt/grl
*
* We acknowledge the good work of the prior contributors :
* Salvatore Virga - salvo.virga@tum.de, Marco Esposito - marco.esposito@tum.de
* Andrew Hundt -
* Carlos J. Rosales - cjrosales@gmail.com
* Enrico Corvaglia
* Marco Esposito - marco.esposito@tum.de
* Manuel Bonilla - josemanuelbonilla@gmail.com
*
* LICENSE :
* Copyright (c) <2018>, <Dinesh Thakur>
* All rights reserved.
*
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   1. Redistributions of source code must retain the above copyright notice,
*   this list of conditions and the following disclaimer.
*
*   2. Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
*   3. Neither the name of the University of Pennsylvania nor the names of its
*   contributors may be used to endorse or promote products derived from this
*   software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <iiwa_hw_fri/iiwa_hw.h>
#include <geometry_msgs/WrenchStamped.h>

using namespace std;

const double kTimeStep = 0.005;
const double kJointLimitSafetyMarginDegree = 3;
#define DEFAULT_PORTID 30200

KukaFRIClient::KukaFRIClient(boost::shared_ptr<IIWA_device> device) : device_(device)
{
  has_entered_command_state_ = false;
  inhibit_motion_in_command_state_ = false;
  utime_last_read_ = -1;
  utime_last_control_ = -1;
  command_valid_ = false;
  once_ = false;
  command_type_ = CommandType::Position;
}

KukaFRIClient::~KukaFRIClient() {}

void KukaFRIClient::onStateChange(KUKA::FRI::ESessionState oldState,
                             KUKA::FRI::ESessionState newState)
{
  KUKA::FRI::LBRClient::onStateChange(oldState, newState);

  const KUKA::FRI::LBRState& state = robotState();
  const uint64_t time = state.getTimestampSec() * 1e6 +
      state.getTimestampNanoSec() / 1e3;

  ROS_INFO_STREAM("onStateChange ( " << time << "): old " << oldState
            << " new " << newState);

  ROS_INFO_STREAM("onStateChange ( " << time
            << "): quality " << state.getConnectionQuality()
            << " safety " << state.getSafetyState()
            << " oper " << state.getOperationMode()
            << " drive " << state.getDriveState()
            << " control " << state.getControlMode()
            << " command " << state.getClientCommandMode()
            << " overlay " << state.getOverlayType());

  if (newState == KUKA::FRI::COMMANDING_ACTIVE)
  {
    joint_position_when_command_entered_.resize(IIWA_JOINTS, 0.);
    std::memcpy(joint_position_when_command_entered_.data(),
                state.getMeasuredJointPosition(),
                IIWA_JOINTS * sizeof(double));

    last_joint_position_command_.resize(IIWA_JOINTS, 0.);
    std::memcpy(last_joint_position_command_.data(),
                state.getMeasuredJointPosition(),
                IIWA_JOINTS * sizeof(double));

    if (!has_entered_command_state_)
    {
      has_entered_command_state_ = true;
      ROS_WARN("FRI in command active state");
    }
    else
    {

      /*
      // We've been in command state before, something bad happened
      // (we did switch to another state, after all), so just stop
      // doing anything.  There's a flag to override this if it's
      // really what the user wants.
      std::cerr << "Re-entering command state." << std::endl;
      if (FLAGS_restart_fri) {
        std::cerr << "Allowing robot motion again due to --restart_fri"
                  << std::endl;
      } else {
        inhibit_motion_in_command_state_ = true;
        std::cerr << "Holding position and ignoring commands.\n"
                  << std::endl;
      }

      */
    }
  }
}

void KukaFRIClient::monitor()
{
  KUKA::FRI::LBRClient::monitor();

  UpdateRobotState(robotState());
}

void KukaFRIClient::waitForCommand()
{
  KUKA::FRI::LBRClient::waitForCommand();

  UpdateRobotState(robotState());
}

void KukaFRIClient::setCommandValid(CommandType command_type)
{

  if(!has_entered_command_state_){
    ROS_WARN("FRI not in commanding state, ignoring");
    ROS_ERROR("cmd %g %g %g %g %g %g %g", device_->joint_position_command[0], device_->joint_position_command[1], device_->joint_position_command[2],
        device_->joint_position_command[3], device_->joint_position_command[4], device_->joint_position_command[5], device_->joint_position_command[6]);
    return;
  }

  command_type_ = command_type;
  command_valid_ = true;
  once_ = true;
}

void KukaFRIClient::command()
{

  UpdateRobotState(robotState());

  double pos[IIWA_JOINTS] = { 0., 0., 0., 0., 0., 0., 0.};

  //Get the current state
  const KUKA::FRI::LBRState& state = robotState();

  //For safety initialize with current position instead of zeros.
  //These should be overwritten below
  std::memcpy(pos,
              state.getMeasuredJointPosition(),
              IIWA_JOINTS * sizeof(double));

  // Current time stamp for this robot.
  const int64_t utime_now =
      state.getTimestampSec() * 1e6 + state.getTimestampNanoSec() / 1e3;

  // Get delta time for this robot.
  double robot_dt = 0.0;
  if (utime_last_control_ != -1)
  {
    robot_dt = (utime_now - utime_last_control_) / 1e6;
    // Check timing
    if (std::abs(robot_dt - kTimeStep) > 1e-3)
      ROS_WARN_STREAM("Warning: dt " << robot_dt << ", kTimeStep " << kTimeStep);
  }
  utime_last_control_ = utime_now;

  if (inhibit_motion_in_command_state_ || !command_valid_) {
    // No command received, just command the position when we
    // entered command state.

    assert(last_joint_position_command_.size() == IIWA_JOINTS);
    memcpy(pos, last_joint_position_command_.data(),
           IIWA_JOINTS * sizeof(double));

    if(once_)
    {
      ROS_ERROR("cmd %g %g %g %g %g %g %g", device_->joint_position_command[0], device_->joint_position_command[1], device_->joint_position_command[2],
        device_->joint_position_command[3], device_->joint_position_command[4], device_->joint_position_command[5], device_->joint_position_command[6]);
      once_ = false;
    }
  } else {
    // Only apply the joint limits when we're responding to command.  If
    // we don't want to command motion, don't change anything.

    if(command_type_ == CommandType::Position)
    {
      std::vector<double> commanded_pos = device_->joint_position_command;

      double current_pos[IIWA_JOINTS] = { 0., 0., 0., 0., 0., 0., 0.};
      std::memcpy(current_pos, state.getMeasuredJointPosition(),IIWA_JOINTS * sizeof(double));

      //Enfore velocity limits, skip current command if exceeded.
      bool valid = false;
      if (robot_dt != 0.)
      {
        valid = true;
        for (int i = 0; i < IIWA_JOINTS; i++)
        {
          double diff = std::abs(commanded_pos[i] - current_pos[i]);
          if (diff > device_->joint_velocity_limits[i]){
            valid = false;
            ROS_ERROR("Command for joint %d exceeds velocity limit", i);
          }
        }
      }

      if(valid)
      {
        //Enforce joint limits with safety margin
        ApplyJointPosLimits(commanded_pos.data());

        memcpy(pos, commanded_pos.data(), IIWA_JOINTS * sizeof(double));
        last_joint_position_command_ = commanded_pos;

        ROS_WARN("Position cmd %g %g %g %g %g %g %g", pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],pos[6]);

      }
      else{
        assert(last_joint_position_command_.size() == IIWA_JOINTS);
        memcpy(pos, last_joint_position_command_.data(),
             IIWA_JOINTS * sizeof(double));
      }
    }
    else if(command_type_ == CommandType::Velocity)
    {
      ROS_WARN("Velocity Command");

      double current_pos[IIWA_JOINTS] = { 0., 0., 0., 0., 0., 0., 0.};
      std::memcpy(current_pos, state.getMeasuredJointPosition(),IIWA_JOINTS * sizeof(double));

      std::vector<double> joint_vel_cmd = device_->joint_velocity_command;
      ROS_WARN("%g %g %g %g %g %g %g", joint_vel_cmd[0], joint_vel_cmd[1], joint_vel_cmd[2],
        joint_vel_cmd[3], joint_vel_cmd[4], joint_vel_cmd[5], joint_vel_cmd[6]);

      std::vector<double> commanded_pos = last_joint_position_command_;

      for (int i = 0; i < IIWA_JOINTS; i++)
      {
        //Get the commanded  velocity
        double cmd_vel = joint_vel_cmd[i];
        double jp = current_pos[i];

        // compute the joint displacement over the current period.
        double a = 1.0; //Currently relative joint accelearation is 0.6*max_accel for safetly. Unkonwn max value.
        double cmd_jp = jp + cmd_vel * robot_dt;// + 0.5*a*dt*dt;

        commanded_pos[i] = cmd_jp;
      }

      //Enfore velocity limits, skip current command if exceeded.
      bool valid = false;
      if (robot_dt != 0.)
      {
        valid = true;
        for (int i = 0; i < IIWA_JOINTS; i++)
        {
          double diff = std::abs(commanded_pos[i] - current_pos[i]);
          if (diff > device_->joint_velocity_limits[i]){
            valid = false;
            ROS_ERROR("Command for joint %d exceeds velocity limit", i);
          }
        }
      }

      if(valid)
      {
        //Enforce joint limits with safety margin
        ApplyJointPosLimits(commanded_pos.data());

        std::vector<double> cmd_pos = last_joint_position_command_;

        //Only set the last joint
        const int a7_index = IIWA_JOINTS - 1;
        cmd_pos[a7_index] = commanded_pos[a7_index];

        memcpy(pos, cmd_pos.data(), IIWA_JOINTS * sizeof(double));
        //last_joint_position_command_ = cmd_pos;

        ROS_WARN("cmd %g %g %g %g %g %g %g", pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],pos[6]);

      }
      else{
        assert(last_joint_position_command_.size() == IIWA_JOINTS);
        memcpy(pos, last_joint_position_command_.data(),
             IIWA_JOINTS * sizeof(double));
      }
    }
    else
    {
      ROS_ERROR("Unknown command_type, setting to last command");
      assert(last_joint_position_command_.size() == IIWA_JOINTS);
      memcpy(pos, last_joint_position_command_.data(),
           IIWA_JOINTS * sizeof(double));
    }

    //Ovveride commands
    ROS_WARN("Ovveride, setting last command");
    assert(last_joint_position_command_.size() == IIWA_JOINTS);
    memcpy(pos, last_joint_position_command_.data(),IIWA_JOINTS * sizeof(double));
    ROS_WARN("cmd %g %g %g %g %g %g %g",pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],pos[6]);

  }
  robotCommand().setJointPosition(pos);

  command_valid_ = false;
}

void KukaFRIClient::UpdateRobotState(const KUKA::FRI::LBRState& state)
{
  // Current time stamp for this robot.
  const int64_t utime_now =
      state.getTimestampSec() * 1e6 + state.getTimestampNanoSec() / 1e3;

  // Get delta time for this robot.
  double robot_dt = 0.0;
  if (utime_last_read_ != -1)
  {
    robot_dt = (utime_now - utime_last_read_) / 1e6;
    // Check timing
    if (std::abs(robot_dt - kTimeStep) > 1e-3)
      ROS_WARN_STREAM("Warning: dt " << robot_dt << ", kTimeStep " << kTimeStep);
  }
  utime_last_read_ = utime_now;

  // Set joint states.
  std::memcpy(device_->joint_position.data(),
              state.getMeasuredJointPosition(), IIWA_JOINTS * sizeof(double));

  std::memcpy(device_->joint_effort.data(),
              state.getMeasuredTorque(), IIWA_JOINTS * sizeof(double));

  // Velocity filtering.
  if (robot_dt != 0.)
  {
    for (int j = 0; j < IIWA_JOINTS; j++)
        device_->joint_velocity[j] = filters::exponentialSmoothing((device_->joint_position[j]-device_->joint_position_prev[j])/robot_dt,
                                                                   device_->joint_velocity[j], 0.2);
  }
  device_->joint_position_prev = device_->joint_position;

}

double KukaFRIClient::ToRadians(double degrees)
{
  return degrees * M_PI / 180.;
}

void KukaFRIClient::ApplyJointPosLimits(double* pos)
{
  const double joint_tol = ToRadians(kJointLimitSafetyMarginDegree);
  for (int i = 0; i < IIWA_JOINTS; i++)
  {
    pos[i] = std::max(std::min(pos[i], (device_->joint_upper_limits[i] - joint_tol)),
                      ((device_->joint_lower_limits[i]) + joint_tol));
  }
}

IIWA_HW::IIWA_HW(ros::NodeHandle nh)
: last_joint_position_command_(7, 0.0)
{
    nh_ = nh;

    timer_ = ros::Time::now();
    control_frequency_ = DEFAULT_CONTROL_FREQUENCY;
    loop_rate_ = new ros::Rate(control_frequency_);

    interface_type_.push_back("PositionJointInterface");
    interface_type_.push_back("VelocityJointInterface");
    interface_type_.push_back("EffortJointInterface");

    params_ = std::make_tuple(
        "Robotiiwa"               , // RobotName,
        "KUKA_LBR_IIWA_14_R820"   , // RobotModel (options are KUKA_LBR_IIWA_14_R820, KUKA_LBR_IIWA_7_R800)
        "0.0.0.0"                 , // LocalUDPAddress
        "30010"                   , // LocalUDPPort
        "172.31.1.147"            , // RemoteUDPAddress
        "192.170.10.100"          , // LocalHostKukaKoniUDPAddress,
        "30200"                   , // LocalHostKukaKoniUDPPort,
        "192.170.10.2"            , // RemoteHostKukaKoniUDPAddress,
        "30200"                   , // RemoteHostKukaKoniUDPPort
        "JAVA"                    , // KukaCommandMode (options are FRI, JAVA)
        "FRI"                       // KukaMonitorMode (options are FRI, JAVA)
    );

    nh_.getParam("RobotName",std::get<RobotName>(params_));
    nh_.getParam("RobotModel",std::get<RobotModel>(params_));
    nh_.getParam("LocalUDPAddress",std::get<LocalUDPAddress>(params_));
    nh_.getParam("LocalUDPPort",std::get<LocalUDPAddress>(params_));
    nh_.getParam("RemoteUDPAddress",std::get<RemoteUDPAddress>(params_));
    nh_.getParam("LocalHostKukaKoniUDPAddress",std::get<LocalHostKukaKoniUDPAddress>(params_));
    nh_.getParam("LocalHostKukaKoniUDPPort",std::get<LocalHostKukaKoniUDPPort>(params_));
    nh_.getParam("RemoteHostKukaKoniUDPAddress",std::get<RemoteHostKukaKoniUDPAddress>(params_));
    nh_.getParam("RemoteHostKukaKoniUDPPort",std::get<RemoteHostKukaKoniUDPPort>(params_));
    nh_.getParam("KukaCommandMode",std::get<KukaCommandMode>(params_));
    nh_.getParam("KukaMonitorMode",std::get<KukaMonitorMode>(params_));

    ready_for_command_ = false;
    ready_for_command_timer_ = ros::Time::now();

    //Amount of time to ignore write commands after controller init/switch
    //ROS control initilizes joint_states to zero and starts commanding zeros on init.
    ignore_command_duration_ = ros::Duration(1.5);
  }

IIWA_HW::~IIWA_HW() {

  if(client_app_)
    client_app_->disconnect();
}

ros::Rate* IIWA_HW::getRate() {
    return loop_rate_;
}

double IIWA_HW::getFrequency() {
    return control_frequency_;
}

void IIWA_HW::setFrequency(double frequency) {
    control_frequency_ = frequency;
    loop_rate_ = new ros::Rate(control_frequency_);
}

bool IIWA_HW::start() {

    // construct a new IIWA device (interface and state storage)
    device_.reset(new IIWA_device());

    // TODO : make use of this
    // get inteface param or give default values
    nh_.param("interface", interface_, std::string("PositionJointInterface"));

    /* TODO
     * nh_.param("move_group", movegroup_name_, "arm");
     * group(movegroup_name_);
     */

    // TODO: use transmission configuration to get names directly from the URDF model
    if ( ros::param::get("joints", device_->joint_names) ) {
        if ( !(device_->joint_names.size() == IIWA_JOINTS) ) {
            ROS_ERROR("This robot has 7 joints, you must specify 7 names for each one");
        }
    } else {
        ROS_ERROR("No joints to be handled, ensure you load a yaml file naming the joint names this hardware interface refers to.");
        throw std::runtime_error("No joint name specification");
    }

    if (!(urdf_model_.initParam("robot_description"))) {
        ROS_ERROR("No URDF model in the robot_description parameter, this is required to define the joint limits.");
        throw std::runtime_error("No URDF model available");
    }

    // initialize and set to zero the state and command values
    device_->init();
    device_->reset();

    // R820 velocity limits
    // A1 - 85 °/s  == 1.483529864195 rad/s
    // A2 - 85 °/s  == 1.483529864195 rad/s
    // A3 - 100 °/s == 1.745329251994 rad/s
    // A4 - 75 °/s  == 1.308996938996 rad/s
    // A5 - 130 °/s == 2.268928027593 rad/s
    // A6 - 135 °/s == 2.356194490192 rad/s
    // A1 - 135 °/s == 2.356194490192 rad/s
    /*
    KukaState::joint_state scaled_velocity_limits;
    scaled_velocity_limits.push_back(85);
    scaled_velocity_limits.push_back(85);
    scaled_velocity_limits.push_back(100);
    scaled_velocity_limits.push_back(75);
    scaled_velocity_limits.push_back(130);
    scaled_velocity_limits.push_back(135);
    scaled_velocity_limits.push_back(135);
    */

    // R800 velocity limits
    // A1 - 75 °/s  == 1.3089969389957472 rad/s
    // A2 - 75 °/s  == 1.3089969389957472 rad/s
    // A3 - 90 °/s == 1.5707963267948966 rad/s
    // A4 - 90 °/s  == 1.5707963267948966 rad/s
    // A5 - 144 °/s == 2.5132741228718345 rad/s
    // A6 - 135 °/s == 2.356194490192345 rad/s
    // A1 - 135 °/s == 2.356194490192345 rad/s

    const double scale_limit = 0.5; //scale the velocity limits to 50% of max
    std::vector<double> scaled_velocity_limits;
    scaled_velocity_limits.push_back( (75 * M_PI/180.0) * scale_limit);
    scaled_velocity_limits.push_back( (75 * M_PI/180.0) * scale_limit);
    scaled_velocity_limits.push_back( (90 * M_PI/180.0) * scale_limit);
    scaled_velocity_limits.push_back( (90 * M_PI/180.0) * scale_limit);
    scaled_velocity_limits.push_back( (144 * M_PI/180.0) * scale_limit);
    scaled_velocity_limits.push_back( (135 * M_PI/180.0) * scale_limit);
    scaled_velocity_limits.push_back( (135 * M_PI/180.0) * scale_limit);

    // general joint to store information
    boost::shared_ptr<const urdf::Joint> joint;

    // create joint handles given the list
    for(int i = 0; i < IIWA_JOINTS; ++i) {
        ROS_INFO_STREAM("Handling joint: " << device_->joint_names[i]);

        // get current joint configuration
        joint = urdf_model_.getJoint(device_->joint_names[i]);
        if(!joint.get()) {
            ROS_ERROR_STREAM("The specified joint "<< device_->joint_names[i] << " can't be found in the URDF model. "
            "Check that you loaded an URDF model in the robot description, or that you spelled correctly the joint name.");
            throw std::runtime_error("Wrong joint name specification");
        }

        // joint state handle
        hardware_interface::JointStateHandle state_handle(device_->joint_names[i],
                                                          &(device_->joint_position[i]),
                                                          &(device_->joint_velocity[i]),
                                                          &(device_->joint_effort[i]));
        state_interface_.registerHandle(state_handle);

        // position command handle
        hardware_interface::JointHandle position_joint_handle = hardware_interface::JointHandle(
            state_interface_.getHandle(device_->joint_names[i]), &device_->joint_position_command[i]);

        position_interface_.registerHandle(position_joint_handle);

        // effort command handle
        hardware_interface::JointHandle effort_joint_handle = hardware_interface::JointHandle(
            state_interface_.getHandle(device_->joint_names[i]), &device_->joint_effort_command[i]);

        effort_interface_.registerHandle(effort_joint_handle);

        registerJointLimits(device_->joint_names[i],
                            position_joint_handle,
                            effort_joint_handle,
                            &urdf_model_,
                            &device_->joint_lower_limits[i],
                            &device_->joint_upper_limits[i],
                            &device_->joint_lower_soft_limits[i],
                            &device_->joint_upper_soft_limits[i],
                            &device_->joint_effort_limits[i]);

        // velocity command handles
        hardware_interface::JointHandle velocity_joint_handle = hardware_interface::JointHandle(
            state_interface_.getHandle(device_->joint_names[i]), &device_->joint_velocity_command[i]);

        velocity_interface_.registerHandle(velocity_joint_handle);

        registerVelocityJointLimits(device_->joint_names[i],
                            velocity_joint_handle,
                            &urdf_model_,
                            &device_->joint_velocity_limits[i]);

        ROS_WARN("%d vel limit from urdf %g, setting to %g", i, device_->joint_velocity_limits[i], scaled_velocity_limits[i]);
        device_->joint_velocity_limits[i] = scaled_velocity_limits[i];
    }

    ROS_INFO("Register state and effort interfaces");

    // TODO: CHECK
    // register ros-controls interfaces
    this->registerInterface(&state_interface_);
    this->registerInterface(&position_interface_);
    this->registerInterface(&velocity_interface_);
    this->registerInterface(&effort_interface_);

    js_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states",100);
    //wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("state/wrench",100);

    current_js_.name = device_->joint_names;

    ROS_INFO_STREAM("Setting up FRI Driver " << std::get<RobotModel>(params_) << " " <<  std::get<KukaCommandMode>(params_) << std::endl);
    ROS_WARN("RobotModel %s", std::get<RobotModel>(params_).c_str());

    fri_client_.reset(new KukaFRIClient(device_));
    udp_connection_.reset(new KUKA::FRI::UdpConnection());
    client_app_.reset(new KUKA::FRI::ClientApplication(*udp_connection_, *fri_client_));

    client_app_->connect(DEFAULT_PORTID, NULL);

    ready_for_command_ = false;
    ready_for_command_timer_ = ros::Time::now();

    return true;
}

void IIWA_HW::registerJointLimits(const std::string& joint_name,
                                  const hardware_interface::JointHandle& position_joint_handle,
                                  const hardware_interface::JointHandle& effort_joint_handle,
                                  const urdf::Model *const urdf_model,
                                  double *const lower_limit, double *const upper_limit,
                                  double *const lower_soft_limit, double *const upper_soft_limit,
                                  double *const effort_limit) {

    *lower_limit = -std::numeric_limits<double>::max();
    *upper_limit = std::numeric_limits<double>::max();
    *lower_soft_limit = -std::numeric_limits<double>::max();
    *upper_soft_limit = std::numeric_limits<double>::max();
    *effort_limit = std::numeric_limits<double>::max();

    joint_limits_interface::JointLimits limits;
    bool has_limits = false;
    joint_limits_interface::SoftJointLimits soft_limits;
    bool has_soft_limits = false;

    if (urdf_model != NULL) {
        const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);

        if (urdf_joint != NULL) {
            // Get limits from the URDF file.
            if (joint_limits_interface::getJointLimits(urdf_joint, limits))
                has_limits = true;

            if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
                has_soft_limits = true;
        }
    }

    if (!has_limits)
        return;

    if (limits.has_position_limits) {
        *lower_limit = limits.min_position;
        *upper_limit = limits.max_position;
    }

    if (limits.has_effort_limits)
        *effort_limit = limits.max_effort;

    if (has_soft_limits) {

        ROS_WARN("Registering Soft limits");

        *lower_soft_limit = soft_limits.min_position;
        *upper_soft_limit = soft_limits.max_position;

        const joint_limits_interface::PositionJointSoftLimitsHandle position_limits_handle(position_joint_handle, limits, soft_limits);
        pj_limits_interface_.registerHandle(position_limits_handle);

        const joint_limits_interface::EffortJointSoftLimitsHandle effort_limits_handle(effort_joint_handle, limits, soft_limits);
        ej_limits_interface_.registerHandle(effort_limits_handle);

    } else {
        const joint_limits_interface::PositionJointSaturationHandle position_sat_handle(position_joint_handle, limits);
        pj_sat_interface_.registerHandle(position_sat_handle);

        const joint_limits_interface::EffortJointSaturationHandle effort_sat_handle(effort_joint_handle, limits);
        ej_sat_interface_.registerHandle(effort_sat_handle);
    }
}

void IIWA_HW::registerVelocityJointLimits(const std::string& joint_name,
                                  const hardware_interface::JointHandle& velocity_joint_handle,
                                  const urdf::Model *const urdf_model,
                                  double *const velocity_limit) {

    *velocity_limit = std::numeric_limits<double>::max();

    joint_limits_interface::JointLimits limits;
    bool has_limits = false;
    joint_limits_interface::SoftJointLimits soft_limits;
    bool has_soft_limits = false;

    if (urdf_model != NULL) {
        const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);

        if (urdf_joint != NULL) {
            // Get limits from the URDF file.
            if (joint_limits_interface::getJointLimits(urdf_joint, limits))
                has_limits = true;

            if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
                has_soft_limits = true;
        }
    }

    if (!has_limits)
        return;

    if (limits.has_velocity_limits)
        *velocity_limit = limits.max_velocity;

    if (has_soft_limits) {

        ROS_ERROR("Registering Velocity Soft limits");

        const joint_limits_interface::VelocityJointSoftLimitsHandle velocity_limits_handle(velocity_joint_handle, limits, soft_limits);
        vj_limits_interface_.registerHandle(velocity_limits_handle);


    } else {
        const joint_limits_interface::VelocityJointSaturationHandle velocity_sat_handle(velocity_joint_handle, limits);
        vj_sat_interface_.registerHandle(velocity_sat_handle);

    }
}

bool IIWA_HW::read(ros::Duration period)
{
  ros::Duration delta = ros::Time::now() - timer_;

  static bool was_connected = false;

  ej_sat_interface_.enforceLimits(period);
  ej_limits_interface_.enforceLimits(period);
  vj_sat_interface_.enforceLimits(period);
  vj_limits_interface_.enforceLimits(period);
  pj_sat_interface_.enforceLimits(period);
  pj_limits_interface_.enforceLimits(period);

  // Joint Position Control
  if (interface_ == "PositionJointInterface")
  {
    if (device_->joint_position_command != last_joint_position_command_)  // avoid sending the same joint command over and over
    {
      if(!ready_for_command_){
        ros::Duration dur = ros::Time::now() - ready_for_command_timer_;

        if(dur.toSec() > ignore_command_duration_.toSec())
          ready_for_command_ = true;

        ROS_WARN("Ignoring command for %g secs %g %g %g %g %g %g %g", (ignore_command_duration_ - dur).toSec(), device_->joint_position_command[0], device_->joint_position_command[1], device_->joint_position_command[2],
        device_->joint_position_command[3], device_->joint_position_command[4], device_->joint_position_command[5], device_->joint_position_command[6]);
      }
      else
      {
        last_joint_position_command_ = device_->joint_position_command;
        fri_client_->setCommandValid(KukaFRIClient::CommandType::Position);
      }
    }
  }
  // Joint Velocity Control
  else if (interface_ == "VelocityJointInterface")
  {
    if(!ready_for_command_){
      ros::Duration dur = ros::Time::now() - ready_for_command_timer_;

      if(dur.toSec() > ignore_command_duration_.toSec())
        ready_for_command_ = true;

      ROS_WARN("Ignoring command for %g secs %g %g %g %g %g %g %g", (ignore_command_duration_ - dur).toSec(), device_->joint_velocity_command[0], device_->joint_velocity_command[1], device_->joint_position_command[2],
      device_->joint_velocity_command[3], device_->joint_velocity_command[4], device_->joint_velocity_command[5], device_->joint_velocity_command[6]);
    }
    else
    {
      //TODO update last_joint_position_command
      //last_joint_position_command_ = device_->joint_position_command;
      fri_client_->setCommandValid(KukaFRIClient::CommandType::Velocity);
    }
  }

  client_app_->step();

  /*
  current_js_.position.clear();
  current_js_.effort.clear();
  current_js_.velocity.clear();

  current_js_.position = device_->joint_position;
  current_js_.effort = device_->joint_effort;
  current_js_.velocity = device_->joint_velocity;

  current_js_.header.stamp = ::ros::Time::now();
  current_js_.header.seq += 1;
  js_pub_.publish(current_js_);
  */

  return 0;
}

bool IIWA_HW::write(ros::Duration period)
{
  return 0;
}

bool IIWA_HW::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                             const std::list<hardware_interface::ControllerInfo>& stop_list) {
  bool ready = true;//hardware_interface::RobotHW::prepareSwitch();

  if(ready)
  {
    ROS_WARN("Start List");
    for (std::list<hardware_interface::ControllerInfo>::const_iterator it=start_list.begin(); it != start_list.end(); ++it)
    {
      std::string start_controller = (*it).name;
      ROS_WARN("name: %s type:%s", start_controller.c_str(), (*it).type.c_str());

      if(start_controller == "PositionJointInterface_trajectory_controller")
      {
        interface_ = "PositionJointInterface";
        ready_for_command_ = false;
        ready_for_command_timer_ = ros::Time::now();
      }
      else if(start_controller == "VelocityJointInterface_trajectory_controller")
      {
        interface_ = "VelocityJointInterface";
        ready_for_command_ = false;
        ready_for_command_timer_ = ros::Time::now();
      }
      else
        ROS_ERROR("Unknown controller requested");
    }

    ROS_WARN("Stop List");
    for (std::list<hardware_interface::ControllerInfo>::const_iterator it=stop_list.begin(); it != stop_list.end(); ++it)
    {
      ROS_WARN("name: %s type:%s", (*it).name.c_str(), (*it).type.c_str());
    }

  }
  return ready;
}