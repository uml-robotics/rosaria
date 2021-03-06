#include <stdio.h>
#include <math.h>
#ifdef ADEPT_PKG
  #include <Aria.h>
#else
  #include <Aria/Aria.h>
#endif
#include <signal.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <sensor_msgs/PointCloud.h>  //for sonar data
#include "nav_msgs/Odometry.h"
#include "rosaria/BumperState.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"  //for tf::getPrefixParam
#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <rosaria/RosAriaConfig.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/Range.h"
#include "rosaria/RangeArray.h"
#include <sstream>
#include <queue>

// Node that interfaces between ROS and mobile robot base features via ARIA library. 
//
// RosAria uses the roscpp client library, see http://www.ros.org/wiki/roscpp for
// information, tutorials and documentation.
class RosAriaNode
{
  public:
    RosAriaNode(ros::NodeHandle n);
    virtual ~RosAriaNode();
    
  public:
    int Setup();
    void cmdvel_cb( const geometry_msgs::TwistConstPtr &);
    //void cmd_enable_motors_cb();
    //void cmd_disable_motors_cb();
    void spin();
    void publish();
    bool hasSonarSubscribers();
    void sonarConnectCb();
    void sonarDisconnectCb();
    void dynamic_reconfigureCB(rosaria::RosAriaConfig &config, uint32_t level);
    void cleanup();
    
  protected:
    ros::NodeHandle n;
    ros::Publisher pose_pub;
    ros::Publisher bumpers_pub;
    ros::Publisher sonar_pub;
    ros::Publisher voltage_pub;
    ros::Publisher range_pub[16];
    ros::Publisher combined_range_pub;

    ros::Publisher recharge_state_pub;
    std_msgs::Int8 recharge_state;

    ros::Publisher state_of_charge_pub;

    ros::Publisher motors_state_pub;
    std_msgs::Bool motors_state;
    bool published_motors_state;

    ros::Subscriber cmdvel_sub;

    ros::Timer sonar_tf_timer;
    void sonarCallback(const ros::TimerEvent &);

    ros::ServiceServer enable_srv;
    ros::ServiceServer disable_srv;
    bool enable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool disable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    void setDynParam(std::string key, double value);
    void setDynParam(std::string key, int value);
    boost::mutex paramqueue_mutex_;
    std::queue<dynamic_reconfigure::Config> paramqueue_;
    void _setDynParams(); //queue work thread
    rosaria::RosAriaConfig dynConf_default;

    ros::Time veltime;

    std::string serial_port;
    int serial_baud;

    ArRobotConnector *conn;
    ArRobot *robot;
    nav_msgs::Odometry position;
    rosaria::BumperState bumpers;
    ArPose pos;
    ArFunctorC<RosAriaNode> myPublishCB;
    ArRobot::ChargeState batteryCharge;

    //for odom->base_link transform
    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;

    rosaria::RangeArray ranges;
    geometry_msgs::TransformStamped sonar_tf_array[16];

    //for resolving tf names.
    std::string tf_prefix;
    std::string frame_id_odom;
    std::string frame_id_base_link;
    std::string frame_id_bumper;
    std::string frame_id_sonar;

    void sonarConnectCB();
    void sonarDisconnectCB();

    //Sonar support
    bool sonars__crossed_the_streams; //true when rear sonars plugged into front port, and front sonars ignored
    bool use_sonar;

    // Debug Aria
    bool debug_aria;
    std::string aria_log_filename;
    
    // Robot Parameters
    int TicksMM, DriftFactor, RevCount;  // Odometry Calibration Settings
    
    // dynamic_reconfigure
    dynamic_reconfigure::Server<rosaria::RosAriaConfig> *dynamic_reconfigure_server;

    boost::thread param_thread_;
};

void RosAriaNode::_setDynParams()
{
  //ROS_INFO("Param set worker started");
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::Config cfg;

  bool empty;
  while(ros::ok())
  {
    if (ros::service::waitForService("~set_parameters", 5000))
    {
      boost::mutex::scoped_lock l(paramqueue_mutex_);
      if (!(empty = paramqueue_.empty())) {
        cfg = paramqueue_.front();
        paramqueue_.pop();
        //ROS_INFO("Popped a dynamicreconfigure config.... need to set it or die trying. %d remaining", paramqueue_.size());
      }
    }
    else
    {
      sleep(0.5);
      continue;
    }
    if (empty)
      break;
    srv_req.config = cfg;
    if (!ros::service::call("~set_parameters", srv_req, srv_resp))
      ROS_ERROR("parameter set failed");
    //else
    //  ROS_INFO("parameter set succeeded");
  }
  ROS_INFO("Param set worker done working");
}

void RosAriaNode::setDynParam(std::string key, int value)
{
  dynamic_reconfigure::IntParameter parm;
  dynamic_reconfigure::Config conf;
  parm.name = key;
  parm.value = value;
  conf.ints.push_back(parm);
  {
    boost::mutex::scoped_lock l(paramqueue_mutex_);
    //ROS_INFO("Enqueueing setting %s to %d.. %d in queue previously", key.c_str(), value, (int)paramqueue_.size());
    bool empty = paramqueue_.empty();
    paramqueue_.push(conf);
    if (empty)
      param_thread_ = boost::thread(&RosAriaNode::_setDynParams, this);
  }
}
void RosAriaNode::setDynParam(std::string key, double value)
{
  dynamic_reconfigure::DoubleParameter parm;
  dynamic_reconfigure::Config conf;
  parm.name = key;
  parm.value = value;
  conf.doubles.push_back(parm);
  {
    boost::mutex::scoped_lock l(paramqueue_mutex_);
    //ROS_INFO("Enqueueing setting %s to %f. %d in queue previously", key.c_str(), value, (int)paramqueue_.size());
    bool empty = paramqueue_.empty();
    paramqueue_.push(conf);
    if (empty)
      param_thread_ = boost::thread(&RosAriaNode::_setDynParams, this);
  }
}

void RosAriaNode::dynamic_reconfigureCB(rosaria::RosAriaConfig &config, uint32_t level)
{
  //ROS_INFO("Dynamic reconfigure callback fired!");
  //
  // Odometry Settings
  //
  robot->lock();
  if(TicksMM != config.TicksMM and config.TicksMM > 0)
  {
    ROS_INFO("Setting TicksMM from Dynamic Reconfigure: %d -> %d ", TicksMM, config.TicksMM);
    TicksMM = config.TicksMM;
    robot->comInt(93, TicksMM);
  }
  
  if(DriftFactor != config.DriftFactor)
  {
    ROS_INFO("Setting DriftFactor from Dynamic Reconfigure: %d -> %d ", DriftFactor, config.DriftFactor);
    DriftFactor = config.DriftFactor;
    robot->comInt(89, DriftFactor);
  }
  
  if(RevCount != config.RevCount and config.RevCount > 0)
  {
    ROS_INFO("Setting RevCount from Dynamic Reconfigure: %d -> %d ", RevCount, config.RevCount);
    RevCount = config.RevCount;
    robot->comInt(88, RevCount);
  }
  
  ////
  ////  WHEN ROSPARAM INITIALIZES WITH THE DEFAULT VALUES, IT FILLS IN ONE NON-ZERO VALUE AT A TIME
  ////  CONSEQUENTLY, THE SAME _CORRECT_ VALUE IS SET TO MAKE SURE THAT parameter_updates, AND ROSPARAM
  ////  ALL REFLECT THE PROGRAMATICALLY SET DEFAULT VALUES AFTER INITIALIZATION COMPLETES
  ////

  //
  // Acceleration Parameters
  //
  double value;
  value = config.trans_accel * 1000.0;
  if(value != robot->getTransAccel() and value > 0)
  {
    ROS_INFO("Setting TransAccel from Dynamic Reconfigure: %f m/s^2", config.trans_accel);
    robot->setTransAccel(value);
  }
  else if (value == 0)
    setDynParam("trans_accel", dynConf_default.trans_accel); 

  value = config.trans_decel * 1000.0;
  if(value != robot->getTransDecel() and value > 0)
  {
    ROS_INFO("Setting TransDecel from Dynamic Reconfigure: %f m/s^2", config.trans_decel);
    robot->setTransDecel(value);
  }
  else if (value == 0)
    setDynParam("trans_decel", dynConf_default.trans_decel);
  
  value = config.rot_accel * 180.0/M_PI;
  if(value != robot->getRotAccel() and value > 0)
  {
    ROS_INFO("Setting RotAccel from Dynamic Reconfigure: %f radians/s^2", config.rot_accel);
    robot->setRotAccel(value);
  }
  else if (value == 0)
    setDynParam("rot_accel", dynConf_default.rot_accel);
  
  value = config.rot_decel * 180.0/M_PI;
  if(value != robot->getRotDecel() and value > 0)
  {
    ROS_INFO("Setting RotDecel from Dynamic Reconfigure: %f radians/s^2", config.rot_decel);
    robot->setRotDecel(value);
  } 
  else if (value == 0)
    setDynParam("rot_decel", dynConf_default.rot_decel);

  value = config.rot_vel_max * 180.0/M_PI;
  if(value != robot->getRotVelMax() and value > 0)
  {
    ROS_INFO("Setting RotVelMax from Dynamic Reconfigure: %f radians/s", config.rot_vel_max);
    robot->setRotVelMax(value);
  }
  else if (value == 0)
    setDynParam("rot_vel_max", dynConf_default.rot_vel_max);

  value=config.trans_vel_max * 1000.0;
  if (value != robot->getTransVelMax() and value > 0)
  {
    ROS_INFO("Setting TransVelMax from Dynamic Reconfigure: %f m/s", config.trans_vel_max);
    robot->setTransVelMax(value);
  }
  else if (value == 0)
    setDynParam("trans_vel_max", dynConf_default.trans_vel_max);

  robot->unlock();
}

bool RosAriaNode::hasSonarSubscribers()
{
    int count = combined_range_pub.getNumSubscribers();
    for (int i = sonars__crossed_the_streams ? 8 : 0; i < 16; i++)
        count += range_pub[i].getNumSubscribers();
    return count > 0;
}

RosAriaNode::RosAriaNode(ros::NodeHandle nh) : 
  myPublishCB(this, &RosAriaNode::publish), serial_port(""), serial_baud(0), sonar_tf_array(), ranges(), param_thread_(), paramqueue_(), paramqueue_mutex_()
{
  // read in config options
  n = nh;
  if (n.hasParam("SonarReassignmentSurgery"))
  {
    n.getParam( "SonarReassignmentSurgery", sonars__crossed_the_streams);
    ROS_INFO("Setting SonarReassignmentSurgery to from ROS Parameter: %d", sonars__crossed_the_streams);
  }
  else
    ROS_INFO("Sonars are not fiddled-with -- YAY");


  // !!! port !!!
  n.param( "port", serial_port, std::string("/dev/ttyUSB0") );
  ROS_INFO( "RosAria: using port: [%s]", serial_port.c_str() );

  n.param("baud", serial_baud, 0);
  if(serial_baud != 0)
  ROS_INFO("RosAria: using serial port baud rate %d", serial_baud);

  // handle debugging more elegantly
  n.param( "debug_aria", debug_aria, false ); // default not to debug
  n.param( "aria_log_filename", aria_log_filename, std::string("Aria.log") );                                                                                      

  // Figure out what frame_id's to use. if a tf_prefix param is specified,
  // it will be added to the beginning of the frame_ids.
  //
  // e.g. rosrun ... _tf_prefix:=MyRobot (or equivalently using <param>s in
  // roslaunch files)
  // will result in the frame_ids being set to /MyRobot/odom etc,
  // rather than /odom. This is useful for Multi Robot Systems.
  // See ROS Wiki for further details.
  frame_id_odom = "odom";
  frame_id_base_link = "base_link";
  frame_id_bumper = "bumpers_frame";
  frame_id_sonar = "sonar_frame";

  ranges.header.frame_id = frame_id_base_link;

  // advertise services for data topics
  // second argument to advertise() is queue size.
  // other argmuments (optional) are callbacks, or a boolean "latch" flag (whether to send current data to new
  // subscribers when they subscribe).
  // See ros::NodeHandle API docs.
}

void RosAriaNode::cleanup()
{
  if (robot != NULL) {
      robot->remSensorInterpTask("ROSPublishingTask");
      // disable motors and sonar.
      robot->disableMotors();
      robot->disableSonar();
      robot->stopRunning();
      delete conn;
      delete robot;
  }
  Aria::shutdown();
}

RosAriaNode::~RosAriaNode()
{
  cleanup();
}

int RosAriaNode::Setup()
{
  // Note, various objects are allocated here which are never deleted (freed), since Setup() is only supposed to be
  // called once per instance, and these objects need to persist until the process terminates.

  robot = new ArRobot();

  ArArgumentBuilder *args = new ArArgumentBuilder(); //  never freed
  ArArgumentParser *argparser = new ArArgumentParser(args); // Warning never freed
  argparser->loadDefaultArguments(); // adds any arguments given in /etc/Aria.args.  Useful on robots with unusual serial port or baud rate (e.g. pioneer lx)

  // Now add any parameters given via ros params (see RosAriaNode constructor):

  // if serial port parameter contains a ':' character, then interpret it as hostname:tcpport
  // for wireless serial connection. Otherwise, interpret it as a serial port name.
  size_t colon_pos = serial_port.find(":");
  if (colon_pos != std::string::npos)
  {
    args->add("-remoteHost"); // pass robot's hostname/IP address to Aria
    args->add(serial_port.substr(0, colon_pos).c_str());
    args->add("-remoteRobotTcpPort"); // pass robot's TCP port to Aria
    args->add(serial_port.substr(colon_pos+1).c_str());
  }
  else
  {
    args->add("-robotPort"); // pass robot's serial port to Aria
    args->add(serial_port.c_str());
  }

  // if a baud rate was specified in baud parameter
  if(serial_baud != 0)
  {
    args->add("-robotBaud");
    char tmp[100];
    snprintf(tmp, 100, "%d", serial_baud);
    args->add(tmp);
  }
  
  if( debug_aria )
  {
    // turn on all ARIA debugging
    args->add("-robotLogPacketsReceived"); // log received packets
    args->add("-robotLogPacketsSent"); // log sent packets
    args->add("-robotLogVelocitiesReceived"); // log received velocities
    args->add("-robotLogMovementSent");
    args->add("-robotLogMovementReceived");
    ArLog::init(ArLog::File, ArLog::Verbose, aria_log_filename.c_str(), true);
  }

  // Connect to the robot
  conn = new ArRobotConnector(argparser, robot); // warning never freed
  if (!conn->connectRobot()) {
    ROS_ERROR("RosAria: ARIA could not connect to robot! (Check ~port parameter is correct, and permissions on port device.)");
    return 1;
  }

  // causes ARIA to load various robot-specific hardware parameters from the robot parameter file in /usr/local/Aria/params
  if(!Aria::parseArgs())
  {
    ROS_ERROR("RosAria: ARIA error parsing ARIA startup parameters!");
    return 1;
  }

  // Start dynamic_reconfigure server
  dynamic_reconfigure_server = new dynamic_reconfigure::Server<rosaria::RosAriaConfig>;

  robot->lock();

  // Setup Parameter Minimums
  rosaria::RosAriaConfig dynConf_min;

  //arbitrary non-zero values so dynamic reconfigure isn't STUPID
  dynConf_min.trans_vel_max = 0.1; 
  dynConf_min.rot_vel_max = 0.1; 
  dynConf_min.trans_accel = 0.1;
  dynConf_min.trans_decel = 0.1;
  dynConf_min.rot_accel = 0.1;
  dynConf_min.rot_decel = 0.1; 
  
  // I'm setting these upper bounds relitivly arbitrarily, feel free to increase them.
  dynConf_min.TicksMM     = 10;
  dynConf_min.DriftFactor = -200;
  dynConf_min.RevCount    = -32760;
  
  dynamic_reconfigure_server->setConfigMin(dynConf_min);
  
  rosaria::RosAriaConfig dynConf_max;
  dynConf_max.trans_vel_max = robot->getAbsoluteMaxTransVel() / 1000.0; 
  dynConf_max.rot_vel_max = robot->getAbsoluteMaxRotVel() *M_PI/180.0; 
  dynConf_max.trans_accel = robot->getAbsoluteMaxTransAccel() / 1000.0;
  dynConf_max.trans_decel = robot->getAbsoluteMaxTransDecel() / 1000.0;
  dynConf_max.rot_accel = robot->getAbsoluteMaxRotAccel() * M_PI/180.0;
  dynConf_max.rot_decel = robot->getAbsoluteMaxRotDecel() * M_PI/180.0;
  
  // I'm setting these upper bounds relitivly arbitrarily, feel free to increase them.
  dynConf_max.TicksMM     = 200;
  dynConf_max.DriftFactor = 200;
  dynConf_max.RevCount    = 32760;
  
  dynamic_reconfigure_server->setConfigMax(dynConf_max);


  dynConf_default.trans_vel_max = robot->getTransVelMax() / 1000.0; 
  dynConf_default.rot_vel_max = robot->getRotVelMax() *M_PI/180.0; 
  dynConf_default.trans_accel = robot->getTransAccel() / 1000.0;
  dynConf_default.trans_decel = robot->getTransDecel() / 1000.0;
  dynConf_default.rot_accel   = robot->getRotAccel() * M_PI/180.0;
  dynConf_default.rot_decel   = robot->getRotDecel() * M_PI/180.0;

/*  ROS_ERROR("ON ROBOT NOW\n\
Trans vel max: %f\n\
Rot vel max: %f\n\
\n\
trans accel: %f\n\
trans decel: %f\n\
rot accel: %f\n\
rot decel: %f", robot->getTransVelMax(), robot->getRotVelMax(), robot->getTransAccel(), robot->getTransDecel(), robot->getRotAccel(), robot->getRotDecel());

  ROS_ERROR("IN DEFAULT CONFIG\n\
Trans vel max: %f\n\
Rot vel max: %f\n\
\n\
trans accel: %f\n\
trans decel: %f\n\
rot accel: %f\n\
rot decel: %f\n", dynConf_default.trans_vel_max,  dynConf_default.rot_vel_max, dynConf_default.trans_accel, dynConf_default.trans_decel, dynConf_default.rot_accel, dynConf_default.rot_decel);*/

  TicksMM = robot->getOrigRobotConfig()->getTicksMM();
  DriftFactor = robot->getOrigRobotConfig()->getDriftFactor();
  RevCount = robot->getOrigRobotConfig()->getRevCount();

  dynConf_default.TicksMM     = TicksMM;
  dynConf_default.DriftFactor = DriftFactor;
  dynConf_default.RevCount    = RevCount;
  
  dynamic_reconfigure_server->setConfigDefault(dynConf_default);

  for(int i = 0; i < 16; i++)
  {
    sonar_tf_array[i].header.frame_id = frame_id_base_link;
    std::stringstream _frame_id;
    _frame_id << "sonar" << i;
    sonar_tf_array[i].child_frame_id = _frame_id.str();
    ArSensorReading* _reading = NULL;
    _reading = robot->getSonarReading(i);
    sonar_tf_array[i].transform.translation.x = _reading->getSensorX() / 1000.0;
    sonar_tf_array[i].transform.translation.y = _reading->getSensorY() / 1000.0;
    sonar_tf_array[i].transform.translation.z = 0.19;
    sonar_tf_array[i].transform.rotation = tf::createQuaternionMsgFromYaw(_reading->getSensorTh() * M_PI / 180.0);
  }

  for (int i=0;i<16;i++) {
      sensor_msgs::Range r;
      ranges.data.push_back(r);
  }

  int i=0,j=0;
  if (sonars__crossed_the_streams) {
    i=8;
    j=8;
  }
  for(; i<16; i++) {
    //populate the RangeArray msg
    std::stringstream _frame_id;
    _frame_id << "sonar" << i;
    ranges.data[i].header.frame_id = _frame_id.str();
    ranges.data[i].radiation_type = 0;
    ranges.data[i].field_of_view = 0.2618f; 
    ranges.data[i].min_range = 0.03f;
    ranges.data[i].max_range = 5.0f;
  }

  // Enable the motors
  robot->enableMotors();

  robot->disableSonar();

  // Initialize bumpers with robot number of bumpers
  bumpers.front_bumpers.resize(robot->getNumFrontBumpers());
  bumpers.rear_bumpers.resize(robot->getNumRearBumpers());

  robot->unlock();

  pose_pub = n.advertise<nav_msgs::Odometry>("pose",1000);
  bumpers_pub = n.advertise<rosaria::BumperState>("bumper_state",1000);

  voltage_pub = n.advertise<std_msgs::Float64>("battery_voltage", 1000);
  
  combined_range_pub = n.advertise<rosaria::RangeArray>("ranges", 1000,
    boost::bind(&RosAriaNode::sonarConnectCb,this),
    boost::bind(&RosAriaNode::sonarDisconnectCb, this));

  for(int i =0; i < 16; i++) {
    std::stringstream topic_name;
    topic_name << "range" << i;
    range_pub[i] = n.advertise<sensor_msgs::Range>(topic_name.str().c_str(), 1000,
      boost::bind(&RosAriaNode::sonarConnectCb,this),
      boost::bind(&RosAriaNode::sonarDisconnectCb, this));
  }
  recharge_state_pub = n.advertise<std_msgs::Int8>("battery_recharge_state", 5, true /*latch*/ );
  recharge_state.data = -2;
  state_of_charge_pub = n.advertise<std_msgs::Float32>("battery_state_of_charge", 100);

  motors_state_pub = n.advertise<std_msgs::Bool>("motors_state", 5, true /*latch*/ );
  motors_state.data = false;
  published_motors_state = false;
 
  // subscribe to services
  cmdvel_sub = n.subscribe( "cmd_vel", 1, (boost::function <void(const geometry_msgs::TwistConstPtr&)>)
    boost::bind(&RosAriaNode::cmdvel_cb, this, _1 ));

  // advertise enable/disable services
  enable_srv = n.advertiseService("enable_motors", &RosAriaNode::enable_motors_cb, this);
  disable_srv = n.advertiseService("disable_motors", &RosAriaNode::disable_motors_cb, this);
 
  veltime = ros::Time::now();
  sonar_tf_timer = n.createTimer(ros::Duration(0.033), &RosAriaNode::sonarCallback, this);
  sonar_tf_timer.stop();

  dynamic_reconfigure_server->setCallback(boost::bind(&RosAriaNode::dynamic_reconfigureCB, this, _1, _2));

  // callback will  be called by ArRobot background processing thread for every SIP data packet received from robot
  robot->addSensorInterpTask("ROSPublishingTask", 100, &myPublishCB);

  // Run ArRobot background processing thread
  robot->runAsync(true);

  return 0;
}

void RosAriaNode::sonarConnectCb()
{
  if (!robot->tryLock()) {
    ROS_ERROR("Skipping sonarConnectCb because could not lock");
    return;
  }
  if (!robot->areSonarsEnabled())
  {
    robot->enableSonar();
    sonar_tf_timer.start();
  }
  robot->unlock();
}

void RosAriaNode::sonarDisconnectCb()
{
  if (!robot->tryLock()) {
    ROS_ERROR("Skipping sonarConnectCb because could not lock");
    return;
  }
  if (robot->areSonarsEnabled())
  {
    robot->disableSonar();
    sonar_tf_timer.stop();
  }
  robot->unlock();
}

void RosAriaNode::spin()
{  
  ros::spin();
}

void RosAriaNode::sonarCallback(const ros::TimerEvent &tick)
{
  ros::Time gameTime = ros::Time::now(); 
  for(int i =0; i < 16; i++)
  {
    sonar_tf_array[i].header.stamp = gameTime;
    odom_broadcaster.sendTransform(sonar_tf_array[i]);
  }
}

void RosAriaNode::publish()
{
  // Note, this is called via SensorInterpTask callback (myPublishCB, named "ROSPublishingTask"). ArRobot object 'robot' sholud not be locked or unlocked.
  pos = robot->getPose();
  tf::poseTFToMsg(tf::Transform(tf::createQuaternionFromYaw(pos.getTh()*M_PI/180), tf::Vector3(pos.getX()/1000,
    pos.getY()/1000, 0)), position.pose.pose); //Aria returns pose in mm.
  position.twist.twist.linear.x = robot->getVel()/1000; //Aria returns velocity in mm/s.
  position.twist.twist.angular.z = robot->getRotVel()*M_PI/180;
  
  position.header.frame_id = frame_id_odom;
  position.child_frame_id = frame_id_base_link;
  position.header.stamp = ros::Time::now();
  pose_pub.publish(position);

  ROS_DEBUG("RosAria: publish: (time %f) pose x: %f, y: %f, angle: %f; linear vel x: %f, y: %f; angular vel z: %f", 
    position.header.stamp.toSec(), 
    (double)position.pose.pose.position.x,
    (double)position.pose.pose.position.y,
    (double)position.pose.pose.orientation.w,
    (double) position.twist.twist.linear.x,
    (double) position.twist.twist.linear.y,
    (double) position.twist.twist.angular.z
  );


  // publishing transform odom->base_link
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = frame_id_odom;
  odom_trans.child_frame_id = frame_id_base_link;
  
  odom_trans.transform.translation.x = pos.getX()/1000;
  odom_trans.transform.translation.y = pos.getY()/1000;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(pos.getTh()*M_PI/180);
  
  odom_broadcaster.sendTransform(odom_trans);
  
  // getStallValue returns 2 bytes with stall bit and bumper bits, packed as (00 00 FrontBumpers RearBumpers)
  int stall = robot->getStallValue();
  unsigned char front_bumpers = (unsigned char)(stall >> 8);
  unsigned char rear_bumpers = (unsigned char)(stall);

  bumpers.header.frame_id = frame_id_bumper;
  bumpers.header.stamp = ros::Time::now();

  std::stringstream bumper_info(std::stringstream::out);
  // Bit 0 is for stall, next bits are for bumpers (leftmost is LSB)
  for (unsigned int i=0; i<robot->getNumFrontBumpers(); i++)
  {
    bumpers.front_bumpers[i] = (front_bumpers & (1 << (i+1))) == 0 ? 0 : 1;
    bumper_info << " " << (front_bumpers & (1 << (i+1)));
  }
  ROS_DEBUG("RosAria: Front bumpers:%s", bumper_info.str().c_str());

  bumper_info.str("");
  // Rear bumpers have reverse order (rightmost is LSB)
  unsigned int numRearBumpers = robot->getNumRearBumpers();
  for (unsigned int i=0; i<numRearBumpers; i++)
  {
    bumpers.rear_bumpers[i] = (rear_bumpers & (1 << (numRearBumpers-i))) == 0 ? 0 : 1;
    bumper_info << " " << (rear_bumpers & (1 << (numRearBumpers-i)));
  }
  ROS_DEBUG("RosAria: Rear bumpers:%s", bumper_info.str().c_str());
  
  bumpers_pub.publish(bumpers);

  //Publish battery information
  // TODO: Decide if BatteryVoltageNow (normalized to (0,12)V)  is a better option
  std_msgs::Float64 batteryVoltage;
  batteryVoltage.data = robot->getRealBatteryVoltageNow();
  voltage_pub.publish(batteryVoltage);

  if(robot->haveStateOfCharge())
  {
    std_msgs::Float32 soc;
    soc.data = robot->getStateOfCharge()/100.0;
    state_of_charge_pub.publish(soc);
  }

  // publish recharge state if changed
  char s = robot->getChargeState();
  if(s != recharge_state.data)
  {
    ROS_INFO("RosAria: publishing new recharge state %d.", s);
    recharge_state.data = s;
    recharge_state_pub.publish(recharge_state);
  }

  // publish motors state if changed
  bool e = robot->areMotorsEnabled();
  if(e != motors_state.data || !published_motors_state)
  {
	ROS_INFO("RosAria: publishing new motors state %d.", e);
	motors_state.data = e;
	motors_state_pub.publish(motors_state);
	published_motors_state = true;
  }

  if (robot->areSonarsEnabled())
  {
    int i = 0;
    int j = 0;
    ArSensorReading* reading = NULL;
    if(sonars__crossed_the_streams)
    {
      i = 8;
      j = 8;
    }
    
    for(; i < 16; i++)
    {
      ranges.data[i].header.stamp = ros::Time::now();
      
      ArSensorReading* _reading = NULL;
      _reading = robot->getSonarReading(i-j);
      ranges.data[i].range = _reading->getRange() / 1000.0f;
      range_pub[i].publish(ranges.data[i]);
    }
    ranges.header.stamp = ros::Time::now();
    combined_range_pub.publish(ranges);
  }  
}

bool RosAriaNode::enable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("RosAria: Enable motors request.");
    robot->lock();
    if(robot->isEStopPressed())
        ROS_WARN("RosAria: Warning: Enable motors requested, but robot also has E-Stop button pressed. Motors will not enable.");
    robot->enableMotors();
    robot->unlock();
	// todo could wait and see if motors do become enabled, and send a response with an error flag if not
    return true;
}

bool RosAriaNode::disable_motors_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("RosAria: Disable motors request.");
    robot->lock();
    robot->disableMotors();
    robot->unlock();
	// todo could wait and see if motors do become disabled, and send a response with an error flag if not
    return true;
}

void
RosAriaNode::cmdvel_cb( const geometry_msgs::TwistConstPtr &msg)
{
  veltime = ros::Time::now();
#ifdef SPEW
  ROS_INFO( "new speed: [%0.2f,%0.2f](%0.3f)", msg->linear.x*1e3, msg->angular.z, veltime.toSec() );
#endif

  robot->lock();
  robot->setVel(msg->linear.x*1e3);
  robot->setRotVel(msg->angular.z*180/M_PI);
  robot->unlock();
  ROS_DEBUG("RosAria: sent vels to to aria (time %f): x vel %f mm/s, y vel %f mm/s, ang vel %f deg/s", veltime.toSec(),
    (double) msg->linear.x * 1e3, (double) msg->linear.y * 1.3, (double) msg->angular.z * 180/M_PI);
}

RosAriaNode *node = NULL;

void mySigintHandler(int sig)
{
  if (node != NULL) {
    node->cleanup();
    delete node;
    node = NULL;
  }

  ros::shutdown();
}

int main( int argc, char** argv )
{
  ros::init(argc,argv, "RosAria");
  ros::NodeHandle n(std::string("~"));
  Aria::init();
  signal(SIGINT, mySigintHandler);

  node = new RosAriaNode(n);

  if( node->Setup() != 0 )
  {
    ROS_FATAL( "RosAria: ROS node setup failed... \n" );  
  }
  else
  {
    node->spin();
  }

  if (node != NULL) {
    node->cleanup();
    delete node;
    node = NULL;
  }

  ROS_INFO( "RosAria: Quitting... \n" );
  return 0;
  
}
