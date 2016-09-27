////////////////////////////////////////////////////////////////////////////////////////////////////
// An adapter that provides the ASRL standard vehicle interface for the Clearpath Horizon Protocol.
//
// Input parameters:
//   twist_output_period_in_s:         how often we should publish output twist messages
//   robot_half_width_in_m:            half-width of the robot - for computing odometry rotation
//   odometry_scaling_multiplier:      what we multiply the encoder values by to get the "true" distance travelled
//
// Input messages:
//   geometry_msgs/Twist:         commands from the path tracker/gamepad
//   clearpath_base/Encoders:     encoder readings from the robot
//   clearpath_base/PowerStatus:  power system readings from the robot
//   clearpath_base/SystemStatus: system readings from the robot
//   clearpath_base/SafetyStatus: safety system readings from the robot
//
// Output messages:
//   geometry_msgs/Twist:        commands sent to the robot
//   nav_msgs/Odometry:          odometry estimates based on wheel encoders
//   asrl_vehicle/VehicleStatus: status messages about the general health of the robot
//
////////////////////////////////////////////////////////////////////////////////////////////////////

// ROS headers
#include <ros/ros.h>

// Utilities
#include <asrl/assert_macros.hpp>

// asrl_vehicle
#include <asrl/vehicle/base_trex.hpp>


// Message types
#include <clearpath_base/Encoders.h>
#include <clearpath_base/PowerStatus.h>
#include <clearpath_base/SafetyStatus.h>
#include <clearpath_base/SystemStatus.h>


namespace asrl { namespace vehicle {
ASRL_DEFINE_EXCEPTION(AsrlHuskyError,std::runtime_error);

class AsrlHusky : public Base
{
public:
  AsrlHusky(ros::NodeHandle nh);

private:
  // Inherited functions
  bool  fillOdometryMessage();
  float getBatteryLevelPercent();
  bool  getIsEStopPressed();
  bool  getAreBrakesOn();
  bool  getIsRobotInErrorState();
  void setVehicleVelocity(double xMetersPerSecond,
                          double yMetersPerSecond,
                          double zMetersPerSecond,
                          double xRadsPerSecond,
                          double yRadsPerSecond,
                          double zRadsPerSecond);
  bool  setOdometryFromPoseWithCovariance(geometry_msgs::PoseWithCovariance pwc);

  // Callbacks
  void callbackEncoders(const clearpath_base::Encoders::ConstPtr & encodersMsg);
  void callbackPowerStatus(const clearpath_base::PowerStatus::ConstPtr & powerStatus);
  void callbackSafetyStatus(const clearpath_base::SafetyStatus::ConstPtr & safetyStatus);
  void callbackSystemStatus(const clearpath_base::SystemStatus::ConstPtr & systemStatus);

  // Publisher function declarations
  void publishTwistMessage(const ros::TimerEvent& t);

  // Configuration parameters
  double robotHalfWidth_, odometryScalingMultiplier_, twistOutputPeriod_, maxMotorTemp_, coolDownTimeMinutes_;
  ros::Duration coolDownTime_;

  // Message variables
  clearpath_base::Encoders lastEncodersMessage_;

  // Odometry state
  bool                 firstEncoderMessage_;
  geometry_msgs::Pose  currentPose_;
  geometry_msgs::Twist currentVelocity_;

  // Speed commands
  double commandedLinearVelocity_, commandedAngularVelocity_;

  // Vehicle status
  float batteryLevelPercent_;
  bool  isEStopPressed_;
  bool  isRobotInErrorState_;
  bool  isOverheated_;
  ros::Time whenOverheated_;

  // ROS stuff
  ros::Publisher  twistPublisher_;
  ros::Timer      twistOutputTimer_;
  ros::Subscriber twistSubscriber_, encodersSubscriber_, powerStatusSubscriber_, safetyStatusSubscriber_, systemStatusSubscriber_;
};


// Constructor
AsrlHusky::AsrlHusky(ros::NodeHandle nh) :
    Base(nh),
    isOverheated_(false)
{
  // parameters
  rosutil::param(nodeHandle_, "twist_output_period_in_s",    twistOutputPeriod_,         0.1);      // s
  rosutil::param(nodeHandle_, "robot_half_width_in_m",       robotHalfWidth_,            0.49/2.0); // m
  rosutil::param(nodeHandle_, "odometry_scaling_multiplier", odometryScalingMultiplier_, 1.11);
  rosutil::param(nodeHandle_, "maximum_motor_temperature", maxMotorTemp_, 60.0); //C
  rosutil::param(nodeHandle_, "motor_cooldown_time", coolDownTimeMinutes_, 30.0); //minutes
  coolDownTime_ = ros::Duration(60*coolDownTimeMinutes_);

  // Twist out messages
  twistOutputTimer_ = nodeHandle_.createTimer(ros::Duration(twistOutputPeriod_), &AsrlHusky::publishTwistMessage, this);
  twistPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("out/twist", 100);

  // subscriber setup
  encodersSubscriber_      = nodeHandle_.subscribe("in/encoders",      100, &AsrlHusky::callbackEncoders,     this);
  powerStatusSubscriber_   = nodeHandle_.subscribe("in/power_status",  100, &AsrlHusky::callbackPowerStatus,  this);
  safetyStatusSubscriber_  = nodeHandle_.subscribe("in/safety_status", 100, &AsrlHusky::callbackSafetyStatus, this);
  systemStatusSubscriber_  = nodeHandle_.subscribe("in/system_status", 100, &AsrlHusky::callbackSystemStatus, this);

  // zero out odometry
  currentPose_.position.x = 0.0;
  currentPose_.position.y = 0.0;
  currentPose_.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // reset encoder reference flag and flag it to get sent out
  firstEncoderMessage_ = true;
}


// Inherited functions
bool AsrlHusky::fillOdometryMessage()
{
  odometryMessage_.header.stamp = ros::Time::now();
  odometryMessage_.pose.pose = currentPose_;
  odometryMessage_.twist.twist = currentVelocity_;

  return true;
}

float AsrlHusky::getBatteryLevelPercent()
{
  return batteryLevelPercent_;
}

bool AsrlHusky::getIsEStopPressed()
{
  return isEStopPressed_;
}

bool AsrlHusky::getAreBrakesOn()
{
  return false; // no brakes on the Husky
}

bool AsrlHusky::getIsRobotInErrorState()
{
  return isRobotInErrorState_;
}

void AsrlHusky::setVehicleVelocity(double xMetersPerSecond,
                                       double yMetersPerSecond,
                                       double zMetersPerSecond,
                                       double xRadsPerSecond,
                                       double yRadsPerSecond,
                                       double zRadsPerSecond)
{
  if (isOverheated_ == false)
  {
    commandedLinearVelocity_  = xMetersPerSecond;
    commandedAngularVelocity_ = zRadsPerSecond;
  }
  else
  {
    commandedLinearVelocity_ = 0;
    commandedAngularVelocity_ = 0;
  }
}

bool AsrlHusky::setOdometryFromPoseWithCovariance(geometry_msgs::PoseWithCovariance pwc)
{
  // Check if the odometry is 2D, if not, post a warning
  tf::Quaternion q(pwc.pose.orientation.x,
                 pwc.pose.orientation.y,
                 pwc.pose.orientation.z,
                 pwc.pose.orientation.w);
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);

  if ( (fabs(roll) > 0.01) || (fabs(pitch) > 0.01) || (fabs(pwc.pose.position.z) > 0.01) )
    ROS_WARN("ASRL_CLEARPATH: Reset odometry command was not 2D. z-values, roll, and pitch are ignored.");

  // Set the pose
  currentPose_.position.x  = pwc.pose.position.x;
  currentPose_.position.y  = pwc.pose.position.y;
  currentPose_.orientation = tf::createQuaternionMsgFromYaw(yaw);

  // reset encoder reference flag
  firstEncoderMessage_ = true;

  return true;
}


// Callbacks
void AsrlHusky::callbackEncoders(const clearpath_base::Encoders::ConstPtr & encodersMsg)
{
  ASRL_ASSERT_EQ(AsrlHuskyError, encodersMsg->encoders.size(), 2, "Number of encoders was not as expected.");

  // Compute change in wheel travel
  double delta_x, delta_theta;

  if (firstEncoderMessage_)
  {
    delta_x     = 0.0;
    delta_theta = 0.0;

    firstEncoderMessage_ = false;
  }
  else
  {
    double left_delta  = odometryScalingMultiplier_ * (encodersMsg->encoders[0].travel - lastEncodersMessage_.encoders[0].travel);
    double right_delta = odometryScalingMultiplier_ * (encodersMsg->encoders[1].travel - lastEncodersMessage_.encoders[1].travel);

    delta_x     = (right_delta + left_delta) / 2.0;
    delta_theta = (right_delta - left_delta) / (2.0*robotHalfWidth_);
  }

  // compute current pose
  double theta = tf::getYaw(currentPose_.orientation) + delta_theta;

  currentPose_.position.x += cos(theta) * delta_x;
  currentPose_.position.y += sin(theta) * delta_x;
  currentPose_.orientation = tf::createQuaternionMsgFromYaw(theta);

  // compute current speed (twist)
  currentVelocity_.linear.x  = odometryScalingMultiplier_ * (encodersMsg->encoders[1].speed + encodersMsg->encoders[0].speed) / 2.0;
  currentVelocity_.angular.z = odometryScalingMultiplier_ * (encodersMsg->encoders[1].speed - encodersMsg->encoders[0].speed) / (2.0*robotHalfWidth_);

  // store the message for the next round
  lastEncodersMessage_ = *encodersMsg;
}

void AsrlHusky::callbackPowerStatus(const clearpath_base::PowerStatus::ConstPtr & powerStatus)
{
  ASRL_ASSERT_EQ(AsrlHuskyError, powerStatus->sources.size(), 1, "Number of power sources was not as expected.");
  batteryLevelPercent_ = (float) (powerStatus->sources[0].charge * 100.0);
}

void AsrlHusky::callbackSafetyStatus(const clearpath_base::SafetyStatus::ConstPtr & safetyStatus)
{
  isEStopPressed_ = safetyStatus->estop;

  if (safetyStatus->flags) // if we have some error code
  {
    isRobotInErrorState_ = true;

    if (safetyStatus->flags & 0x01)
    {
      // HORIZON_TIMEOUT: The system motion control has not received a Horizon motion command in the last second.
      ROS_WARN("Clearpath error state: HORIZON_TIMEOUT: no motion input commands");
    }

    if (safetyStatus->flags & 0x04)
    {
      // NORC: The system has a remote control module plugged in but is not currently receiving a valid signal.
      ROS_WARN("Clearpath error state: NORC: remote control module plugged in, but no signal");
    }

    if (safetyStatus->flags & 0x10)
    {
      // HORIZON_PAUSE: A remote pause has been set via command 0x0010 - Set Safety System.
      ROS_WARN("Clearpath error state: HORIZON_PAUSE: remote pause set");
    }
  }
  else
  {
    isRobotInErrorState_ = false;
  }
}

void AsrlHusky::callbackSystemStatus(const clearpath_base::SystemStatus::ConstPtr & systemStatus)
{
  //If not currently overheated and temperatures reach maxMotorTemp_, mark as overheated (i.e., catch rising edges only)
  if ((isOverheated_ == false) && ((systemStatus->temperatures[2] > maxMotorTemp_) || (systemStatus->temperatures[3] > maxMotorTemp_)) )
  { //Record time and send into thermal protection.
    isOverheated_ = true;
    whenOverheated_ = ros::Time::now();
    ROS_WARN("Clearpath temperature protection triggered at %fC (Left: %fC, Right: %fC).", maxMotorTemp_, systemStatus->temperatures[2], systemStatus->temperatures[3]);
  }
  //If currently isOverheated_ and still in coolDownTime_, wait.
  else if ( (isOverheated_ == true) && (ros::Time::now() - whenOverheated_) <= coolDownTime_)
  { //Output a useful warning message and continue to wait.
    ROS_INFO("Clearpath temperature protection active until %f (Left: %fC, Right: %fC).", (whenOverheated_ + coolDownTime_).toSec(), systemStatus->temperatures[2], systemStatus->temperatures[3]);
  }
  //If currently isOverheated_ yet coolDownTime_ has passed, disable temperature protection
  else if ( (isOverheated_ == true) && (ros::Time::now() - whenOverheated_) > coolDownTime_)
  {
    isOverheated_ = false;
    ROS_INFO("Clearpath temperature protection finished (Left: %fC, Right: %fC).", systemStatus->temperatures[2], systemStatus->temperatures[3]);
  }
  else
  {
    isOverheated_ = false;
  }
}

// Publisher function definitions
void AsrlHusky::publishTwistMessage(const ros::TimerEvent& t)
{
  geometry_msgs::Twist twist;
  twist.linear.x  = commandedLinearVelocity_;
  twist.angular.z = commandedAngularVelocity_;
  twistPublisher_.publish(twist);
}

}} // end namespace asrl::vehicle



int main(int argc, char** argv)
{
  return asrl::rosutil::spinNodeCatchException<asrl::vehicle::AsrlHusky>(argc, argv, "asrl_clearpath");
}

