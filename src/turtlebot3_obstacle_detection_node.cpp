#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>

#include <tf/tf.h>
#include <stdio.h>
#include <vector>
#include <cmath> // isnan()
#include <bitset>

double DEG2RAD(int deg){
  return (M_PI / 180) * deg;
}

// obstacle sections : 4 sections 
enum{
  RIGHT = 0,
  FRIGHT = 1,
  FLEFT = 2,
  LEFT = 3,
  OBS_SECTION_SIZE
};

// obstacle state
enum{
  OBS_CLEAR = 0,

  OBS_L=1,    // 1000
  OBS_R=2,     // 0001
  OBS_L_R=3,  // 1001
  OBS_FL=4,   // 0100
  OBS_FR=5,   // 0010
  OBS_F=6,    // 0110
  OBS_L_FL=7, // 1100
  OBS_FR_R=8, // 0011
  OBS_L_F=9,  // 1110
  OBS_F_R=10, // 0111
  OBS_ALL=11, // 1111
  OBS_STATE_SIZE,// =12
};

const char *OBS_STRING[]=
{
  "OBS_CLEAR" ,
  "OBS_L",
  "OBS_R"    ,
  "OBS_L_R",
  "OBS_FL",
  "OBS_FR",
  "OBS_F"  ,
  "OBS_L_FL" ,
  "OBS_FR_R" ,
  "OBS_L_F" ,
  "OBS_F_R" ,
  "OBS_ALL" ,
  "OBS_STATE_SIZE"
};

// drive state ----------------
// DRIVE : go straight (linear vel: 0.1)
// DRIVE_APPROACH : slowly (0.05)
// DRIVE_AVOID : avoid moving
enum{
  DRIVE = 0,
  DRIVE_APPROACH = 1,
  DRIVE_AVOID = 2,
};

// parameters struct
struct ODSParameters
{
  double max_range; // maximum detecting range
  double min_range; // minimum detecting range
  double section_angle[OBS_SECTION_SIZE]; // each sections' angle
  double obs_dist_threshold[OBS_SECTION_SIZE]; // distance threshold of detecting obstacle
  int obs_count_threshold[OBS_SECTION_SIZE]; // count threshold of detecting obstacle
  double drive_linvel; // normal linvel
  double approach_linvel; // approaching linvel
  double reset_linvel; // resetting linvel
  double avoid_angvel; // avoiding angvel
  int initial_centi; // at run(), how distance to go?
  double error;
};


class ODS{
public:
ODS();
~ODS();
    // void init(const geometry_msgs::Pose2D& _init_pose);
    void init();
    void run();

    //LaserScan
    void initScan(const sensor_msgs::LaserScanConstPtr& _scan);
    void scanCallBack(const sensor_msgs::LaserScanConstPtr& _scan);

    //Twist
    void publishTwist(const double _lin_vel, const double _ang_vel);

    //Odom
    void odometryCallBack(const nav_msgs::Odometry::ConstPtr _odom);

    //Obstacle Detection
    bool is_ObsDetected(const std::vector<double>& _ranges);
    void scanObsSections(const std::vector<double>& _ranges);

    /**
     * At first, go forward centi, then turn 90 degree.
     * */
    void reachingObs(int centi);

private:
    ros::NodeHandle nh_;

    //LaserScan
    ros::Subscriber scan_sub_;
    //pose2D
    ros::Subscriber odom_sub_; 
    //Twist
    ros::Publisher twist_pub_;
    ros::Publisher pose_pub_;

    double scan_angle_min_;
    double scan_angle_max_;
    double scan_angle_inc_;
  

    //parameter struct
    ODSParameters params_;

    // bool variables
    bool is_first_scan_;
    bool is_first_yaw_;

    //obstacle
    double obs_dist_[OBS_SECTION_SIZE];
    int obs_count_[OBS_SECTION_SIZE];
    std::bitset<4> obs_bits_;
    int obs_state_;

    //yaw
    double initial_yaw_;
    double current_yaw_;

    //driving state
    int driving_state_;
};

// constructor & destructor
ODS::ODS(){
    is_first_scan_ = true;
    is_first_yaw_ = true;
    obs_bits_.reset();
    for(int i = 0; i< OBS_SECTION_SIZE; i++){
      obs_count_[i] = 0;
    }
    driving_state_ = DRIVE;
}
ODS::~ODS(){

}


//init params
void ODS::init(){
    ros::NodeHandle nh("~");
     // would be updated while obstacle detection.
    nh.param<double>("max_range",params_.max_range, 2.0);
    nh.param<double>("min_range",params_.min_range, 0.0);

    nh.param<double>("right_section_angle",params_.section_angle[RIGHT],  360.0-90.0); // 80 --> 90
    nh.param<double>("fright_section_angle",params_.section_angle[FRIGHT],360.0-40.0); // 45 --> 40
    nh.param<double>("fleft_section_angle",params_.section_angle[FLEFT],  40.0);
    nh.param<double>("left_section_angle",params_.section_angle[LEFT],    90.0);        //80 --> 90

    nh.param<double>("obs_dist_thresh_r", params_.obs_dist_threshold[RIGHT],  0.21);      // all 0.21 --> 23
    nh.param<double>("obs_dist_thresh_fr", params_.obs_dist_threshold[FRIGHT],0.21);
    nh.param<double>("obs_dist_thresh_fl", params_.obs_dist_threshold[FLEFT], 0.21);
    nh.param<double>("obs_dist_thresh_l", params_.obs_dist_threshold[LEFT],   0.21);

    nh.param<int>("obs_count_thresh_r", params_.obs_count_threshold[RIGHT],1);
    nh.param<int>("obs_count_thresh_fr", params_.obs_count_threshold[FRIGHT],1);
    nh.param<int>("obs_count_thresh_fl", params_.obs_count_threshold[FLEFT],1);
    nh.param<int>("obs_count_thresh_l", params_.obs_count_threshold[LEFT],1);
  
    nh.param<double>("drive_linvel",params_.drive_linvel, 0.1);
    nh.param<double>("reset_linvel",params_.reset_linvel, 0.06);
    nh.param<double>("approach_linvel",params_.approach_linvel, 0.02);
    nh.param<double>("avoid_linvel",params_.avoid_angvel, M_PI / 7);

    nh.param<int>("centimeter",params_.initial_centi, 50);

    nh.param<double>("error",params_.error, 0.05);
}
void ODS::run(){
    int queue_size = 50;

    scan_sub_ = nh_.subscribe("scan", queue_size, &ODS::scanCallBack, this);
    odom_sub_ = nh_.subscribe("odom", 1, &ODS::odometryCallBack,this);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",queue_size,true); // true?
    pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>("pose2D",1);

    reachingObs(params_.initial_centi);
}

void ODS::reachingObs(int centi){
  ros::Duration five_sec(5.0);
  ros::Duration two_sec(2.0);
  ros::Duration one_sec(1.0);

  ros::Rate freq(10);

  ros::Time now;
  ros::Time time = ros::Time::now();

  while((now = ros::Time::now()) <= time + five_sec){
    publishTwist(params_.drive_linvel, 0.0);
    ROS_INFO("reachingObs : go forward");
    freq.sleep();
  }

  time = ros::Time::now();
  while((now = ros::Time::now()) <= time + two_sec){
    publishTwist(0.0, M_PI/4);
    ROS_INFO("reachingObs : turn 90 degree");
    freq.sleep();
  }

  time = ros::Time::now();
  while((now = ros::Time::now()) <= time + one_sec){
    publishTwist(0.0,0.0);
    ROS_INFO("reachingObs : stop");
    freq.sleep();
  }
  
}

void ODS::publishTwist(const double _lin_vel, const double _ang_vel){
    geometry_msgs::Twist vel;
    vel.linear.x = _lin_vel;
    vel.angular.z = _ang_vel;
    twist_pub_.publish(vel);
}

void ODS::initScan(const sensor_msgs::LaserScanConstPtr& _scan){
    scan_angle_min_ = _scan->angle_min;
    scan_angle_max_ = _scan->angle_max;
    scan_angle_inc_ = _scan->angle_increment;

    ROS_INFO( "min: %.3f, max: %.3f, inc: %.3f\n",
      scan_angle_min_, scan_angle_max_, scan_angle_inc_ );
}


void ODS::odometryCallBack(const nav_msgs::Odometry::ConstPtr _odom){
  geometry_msgs::Pose2D pose2d;
  pose2d.x = _odom->pose.pose.position.x;
  pose2d.y = _odom->pose.pose.position.y;

  tf::Quaternion q(
    _odom->pose.pose.orientation.x,
    _odom->pose.pose.orientation.y,
    _odom->pose.pose.orientation.z,
    _odom->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll,pitch,yaw;
  m.getRPY(roll,pitch,yaw);

  pose2d.theta = yaw;
  if(is_first_yaw_){
    initial_yaw_ = yaw;
    is_first_yaw_ = false;
  }
  current_yaw_ = yaw;
  // ROS_INFO("yaw: %f", yaw);
  pose_pub_.publish(pose2d);
}


void ODS::scanObsSections(const std::vector<double>& _ranges){
  // determine boundaries
  const double FRONT_MIN_RAD = DEG2RAD(params_.section_angle[FLEFT]);
  const double FRONT_MAX_RAD = DEG2RAD(params_.section_angle[FRIGHT]);
  const double LEFT_RAD = DEG2RAD(params_.section_angle[LEFT]);
  const double RIGHT_RAD = DEG2RAD(params_.section_angle[RIGHT]);
  const int front_min_idx = (FRONT_MIN_RAD - scan_angle_min_) / scan_angle_inc_;
  const int front_max_idx = (FRONT_MAX_RAD - scan_angle_min_) / scan_angle_inc_;
  const int left_idx = (LEFT_RAD - scan_angle_min_) / scan_angle_inc_;
  const int right_idx = (RIGHT_RAD - scan_angle_min_) / scan_angle_inc_;
  const int front_mid_idx = _ranges.size();
  // ROS_INFO("min rad = %f, max rad = %f",FRONT_MIN_RAD,FRONT_MAX_RAD);
  // ROS_INFO("min idx = %d, max_idx = %d, front_mid_idx %d, left idx = %d, right idx = %d",front_min_idx,front_max_idx,front_mid_idx,left_idx,right_idx);
  

  ROS_ASSERT( front_max_idx < _ranges.size());
  ROS_ASSERT( front_min_idx >= 0);

  //// fl section
  // read threshold
  double min_dist = params_.obs_dist_threshold[FLEFT];
  for(int i=0; i<front_min_idx;i++){  // 0 ~ 30
    if(_ranges[i] < min_dist && _ranges[i] != 0.0){
      // if(_ranges[i] < min_dist){
      obs_bits_.set(FLEFT);
      min_dist = _ranges[i];
    }
  }
  // if detected, update detected count and distance to obstacle
  if(obs_bits_.test(FLEFT)){
    obs_count_[FLEFT]++;
    //update minimum distance
    obs_dist_[FLEFT] = min_dist;
  }
  else{
    obs_count_[FLEFT] = 0;
    //set maximum range for no obstacle case
    obs_dist_[FLEFT] = params_.max_range;
  }

  //// fr section
  //read threshold
  min_dist = params_.obs_dist_threshold[FRIGHT];
  for(int i=front_max_idx; i<front_mid_idx;i++){   // 330 ~ 360
    if(_ranges[i] < min_dist&& _ranges[i] != 0.0){
      // if(_ranges[i] < min_dist){
      obs_bits_.set(FRIGHT);
      //update minimum distance
      min_dist = _ranges[i];
    }
  }
  // if detected, update detected count and distance to obstacle
  if(obs_bits_.test(FRIGHT)){
    obs_count_[FRIGHT]++;
    obs_dist_[FRIGHT] = min_dist;
  }
  else{
    obs_count_[FRIGHT] = 0;
    //set maximum range for no obstacle case
    obs_dist_[FRIGHT] = params_.max_range;
  }

  //// left section
  //read threshold
  min_dist = params_.obs_dist_threshold[LEFT];
  for(int i=front_min_idx; i<left_idx;i++){     //30 ~ 80
    if(_ranges[i] < min_dist&& _ranges[i] != 0.0){
      // if(_ranges[i] < min_dist){
      obs_bits_.set(LEFT);
      //update minimum distance
      min_dist = _ranges[i];
    }
  }
  // if detected, update detected count and distance to obstacle
  if(obs_bits_.test(LEFT)){
    obs_count_[LEFT]++;
    obs_dist_[LEFT] = min_dist;
  }
  else{
    obs_count_[LEFT] = 0;
    //set maximum range for no obstacle case
    obs_dist_[LEFT] = params_.max_range;
  }

  //// right section
  //read threshold
  min_dist = params_.obs_dist_threshold[RIGHT];
  for(int i=right_idx; i<front_max_idx;i++){
    if(_ranges[i] < min_dist&& _ranges[i] != 0.0){
      // if(_ranges[i] < min_dist){
      obs_bits_.set(RIGHT);
      //update minimum distance
      min_dist = _ranges[i];
    }
  }
  // if detected, update detected count and distance to obstacle
  if(obs_bits_.test(RIGHT)){
    obs_count_[RIGHT]++;
    obs_dist_[RIGHT] = min_dist;
  }
  else{
    obs_count_[RIGHT] = 0;
    //set maximum range for no obstacle case
    obs_dist_[RIGHT] = params_.max_range;
  }

  

  if(obs_bits_.test(FLEFT) && obs_bits_.test(FRIGHT))
    {
    int i,j;
    for(i=0; i<front_min_idx;i++){
      for(j=front_mid_idx-1; j>=front_max_idx;j--){ 
        if(_ranges[i] > _ranges[j]+params_.error) {
          obs_bits_.reset(FLEFT);
          obs_bits_.reset(LEFT);  //?
          ROS_INFO("*****ranges[%d]: %.3f, ranges[%d]: %.3f",i,_ranges[i],j,_ranges[j]);
          ROS_INFO("reset FLEFT");
          break;
        }
        else if (_ranges[i] < _ranges[j]-params_.error) {
          obs_bits_.reset(FRIGHT);
          obs_bits_.reset(RIGHT); // ??
          ROS_INFO("****ranges[%d]: %.3f, ranges[%d]: %.3f",i,_ranges[i],j,_ranges[j]);
          ROS_INFO("reset FRIGHT");
          break;
        }
        // ROS_INFO("ranges[%d]: %.3f, ranges[%d]: %.3f",i,_ranges[i],j,_ranges[j]);
        i++;
      }
      break;
    }
  }
  if(obs_bits_.test(RIGHT) && obs_bits_.test(LEFT))  
      {
    int i,j;
    for(i=front_min_idx; i<left_idx;i++){
      for(j=front_max_idx; j>=right_idx;j--){ 
        if(_ranges[i] > _ranges[j]+params_.error) {
          obs_bits_.reset(LEFT);

            ROS_INFO("***********ranges[%d]: %.3f, ranges[%d]: %.3f",i,_ranges[i],j,_ranges[j]);
             ROS_INFO("reset LEFT");
             
          break;
        }
        else if (_ranges[i] < _ranges[j]-params_.error) {
          obs_bits_.reset(RIGHT);
          ROS_INFO("*************ranges[%d]: %.3f, ranges[%d]: %.3f",i,_ranges[i],j,_ranges[j]);
          ROS_INFO("reset LEFT");
          break;
        }
        // ROS_INFO("ranges[%d]: %.3f, ranges[%d]: %.3f",i,_ranges[i],j,_ranges[j]);        
        i++;
      }
      break;
    }
  }

  if((obs_bits_.test(LEFT) && obs_bits_.test(FLEFT) && obs_bits_.test(FRIGHT) && obs_bits_.test(RIGHT))){
       int i,j;
    for(i=0; i<left_idx;i++){
      for(j=front_mid_idx-1; j>=right_idx;j--){ 
        if(_ranges[i] > _ranges[j]+params_.error) {
          obs_bits_.reset(LEFT);
          obs_bits_.reset(FLEFT);
          break;
        }
        else if (_ranges[i] < _ranges[j]-params_.error) {
          obs_bits_.reset(RIGHT);
          obs_bits_.reset(FRIGHT);
          break;
        }
        i++;
      }
      break;
    }
  }
  
}

bool ODS::is_ObsDetected(const std::vector<double>& _ranges){
    //clear previous obstacle bits and state
    obs_bits_.reset();
    obs_state_ = OBS_CLEAR;
    //scan obstacles
    scanObsSections(_ranges);
    
    bool detected = false;

    // check each sections' obstacle count
    for(int i=0; i<OBS_SECTION_SIZE;i++){
      // ROS_INFO("obs_count_[%d] = %d \t",i,obs_count_[i]);
      if(obs_count_[i] > params_.obs_count_threshold[i]){
        detected = true;
        break;
      }
    }
    //return false if not detected
    if(!detected) return false;

    //// determine obstacle statefleft
    // left section ( 1000 = 0x8 )
    if( obs_bits_ == std::bitset<OBS_SECTION_SIZE>(0x8) ){
      obs_state_ = OBS_L;
    }
    // right section ( 0001 = 0x1 )
    else if( obs_bits_ == std::bitset<OBS_SECTION_SIZE>(0x1)){
      obs_state_ = OBS_R;
    }
    // left, right section ( 1001 = 0x9 )
    else if( obs_bits_ == std::bitset<OBS_SECTION_SIZE>(0x9)){
      obs_state_ = OBS_L_R;
    }
    // fleft section ( 0100 = 0x4 ) // 0101 . 0x5
    else if( obs_bits_ == std::bitset<OBS_SECTION_SIZE>(0x4)
          || obs_bits_ == std::bitset<OBS_SECTION_SIZE>(0x5)){
      obs_state_ = OBS_FL;
    }
    // fright section ( 0010 = 0x2 ) // 1010
    else if( obs_bits_ == std::bitset<OBS_SECTION_SIZE>(0x2)
          || obs_bits_ == std::bitset<OBS_SECTION_SIZE>(0xa)){
      obs_state_ = OBS_FR;
    }
    // front section ( 0110 = 0x6 )
    else if( obs_bits_ == std::bitset<OBS_SECTION_SIZE>(0x6)){    // 0110
      obs_state_ = OBS_F;
    }
    // left, fleft section ( 1100 = 0xc ), 1101
    else if( obs_bits_ == std::bitset<OBS_SECTION_SIZE>(0xc)
            || obs_bits_ == std::bitset<OBS_SECTION_SIZE>(0xd)){
      obs_state_ = OBS_L_FL;
    }
    // fright, right section ( 0011 = 0x3 ), 1011
    else if( obs_bits_ == std::bitset<OBS_SECTION_SIZE>(0x3)
          || obs_bits_ == std::bitset<OBS_SECTION_SIZE>(0xb)){
      obs_state_ = OBS_FR_R;
    }
    // left, front section ( 1110 = 0xe )
    else if( obs_bits_ == std::bitset<OBS_SECTION_SIZE>(0xe)){
      obs_state_ = OBS_L_F;
    }
    // front, right section ( 0111 = 0x7 )
    else if( obs_bits_ == std::bitset<OBS_SECTION_SIZE>(0x7)){
      obs_state_ = OBS_F_R;
    }

    else
    {
      obs_state_ = OBS_ALL;
    }

    return true;
}

void ODS::scanCallBack(const sensor_msgs::LaserScanConstPtr& _scan){
    //initializing LaserScan
    if(is_first_scan_){
      initScan(_scan);
      is_first_scan_ = false;
      ROS_INFO("First SCAN FINISHED");
      return;
    }
    
    // filtering ranges
    std::vector<double> ranges;
    for( int i = 0; i < _scan->ranges.size(); i++){
      if(_scan->ranges[i] <= params_.min_range || _scan->ranges[i] > params_.max_range || std::isnan(_scan->ranges[i])){
        ranges.push_back(params_.max_range);
      }
      else{
        ranges.push_back(_scan->ranges[i]);
      }
    }

    //determine obstacle state
    bool detected = is_ObsDetected(ranges);
    // for(int i = 0; i<ranges.size();i++){
    //   printf("[%d]: %.3f, ", i,ranges[i]);
    // }
    ROS_INFO("detected? %d, obstacle state= %s", detected, OBS_STRING[obs_state_]);
    ROS_INFO( "obs_dist left: %.2f, fleft: %.2f, fright: %.2f, right: %.2f", obs_dist_[LEFT], obs_dist_[FLEFT], obs_dist_[FRIGHT], obs_dist_[RIGHT]);

    bool is_initial_state = true;

    switch(driving_state_){
      case DRIVE_AVOID:
      if(detected){ //>> not necessary
        if (obs_state_ == OBS_FL || obs_state_ == OBS_L_FL || obs_state_ == OBS_L_F){
            publishTwist(0.0, -params_.avoid_angvel);
            ROS_INFO("turning.. CW");
            is_initial_state = false; 
        }
        else if (obs_state_ == OBS_FR || obs_state_ == OBS_FR_R || obs_state_ == OBS_F_R){
            publishTwist(0.0, params_.avoid_angvel);
            ROS_INFO("turning.. CCW");
            is_initial_state = false;
        }
        // else if(obs_state_ == OBS_L || obs_state_ == OBS_CLEAR || obs_state_ == OBS_R){
        else{
          publishTwist(0.0,0.0);     
          ROS_INFO("no twist");
          driving_state_ = DRIVE;
        }
      }
      break;

      case DRIVE:
      if(obs_state_ == OBS_R || obs_state_ == OBS_L){       
          publishTwist(params_.drive_linvel,0.0);
          ROS_INFO("drive");
      }
      else if(obs_state_ == OBS_CLEAR){
        int iniY = initial_yaw_ * 5;
        int curY = current_yaw_ * 5;

        is_initial_state = (iniY == curY);

        ROS_INFO("initial_yaw_: %d, current: %d",iniY,curY);

        if(!is_initial_state){
            if(iniY > curY) {
              publishTwist(params_.reset_linvel,params_.avoid_angvel);
              ROS_INFO("init > current, turning ccw");
            }
            else if(iniY < curY) {
              publishTwist(params_.reset_linvel,-params_.avoid_angvel);
              ROS_INFO("init < current, turning cw");
            }
        }
        else{
          publishTwist(params_.drive_linvel,0.0);
          ROS_INFO("iniY == curY, drive!");
        }
      }
      else if (obs_state_ == OBS_F || obs_state_ == OBS_L_R){
        publishTwist(params_.approach_linvel,0.0);
        driving_state_ = DRIVE_AVOID;
        ROS_INFO("Approaching.. to AVOID MODE");
      }
      else{
        publishTwist(0.0,0.0);
        driving_state_ = DRIVE_AVOID;
      }
    }
}

int main(int argc, char** argv){
    // Init ROS node
  ros::init(argc, argv, "obs_detection");

  ODS ods;
  ods.init();
  ods.run();

  try{
    ros::spin();
  }
  catch(std::runtime_error& e){
    ROS_ERROR( "ros spin failed: %s", e.what());
    return -1;
  }
  
  return 0;

}
  

