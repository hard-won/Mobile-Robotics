#include <chrono>
#include <functional>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <fstream>
#include <time.h>
#include <iomanip>

using namespace std::chrono_literals;
using namespace std;
ofstream odomTrajFile; // Declare a file object to record odometry data.
ofstream odomVelFile; // Declare a file object to record velocity data.
ofstream laserFile; // Declare a file object for recording your laser data.
class Stopper : public rclcpp::Node{
public:
/* velocity control variables*/
constexpr const static double FORWARD_SPEED_LOW = 0.1;
constexpr const static double FORWARD_SPEED_MIDDLE = 0.3;
constexpr const static double FORWARD_SPEED_HIGH = 0.5;
constexpr const static double FORWARD_SPEED_STOP = 0;
constexpr const static double TURN_LEFT_SPEED_LOW = 0.3;
constexpr const static double TURN_LEFT_SPEED_MIDDLE = 0.6;
constexpr const static double TURN_LEFT_SPEED_HIGH = 1.0;
constexpr const static double TURN_RIGHT_SPEED_LOW = -0.3;
constexpr const static double TURN_RIGHT_SPEED_MIDDLE = -0.6;
constexpr const static double TURN_RIGHT_SPEED_HIGH = -1.0;
/* class constructor */
Stopper():Node("Stopper"), count_(0){
publisher_ = this->create_publisher< geometry_msgs::msg::Twist>("cmd_vel", 10);
odomSub_=this->create_subscription<nav_msgs::msg::Odometry>("odom",10,
std::bind(&Stopper::odomCallback, this, std::placeholders::_1));
laserScan_=this->create_subscription<sensor_msgs::msg::LaserScan>("scan",10,
std::bind(&Stopper::scanCallback, this, std::placeholders::_1));
};
/* moving functions */
void startMoving();
void moveStop();
void moveForward(double forwardSpeed);
void moveRight(double turn_right_speed);
void moveForwardRight(double forwardSpeed, double turn_right_speed);
void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
double PositionX=0.3, PositionY=0.3, homeX=0.3, homeY=0.3;
double odom_landmark1=1.20, odom_landmark1a=0.38, odom_landmark2=0.80;
int stage=0;
double robVelocity;
int numberOfCycle=0;
void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
double frontRange, mleftRange, leftRange, rightRange, mrightRange;
int laser_index = 0; // index the laser scan data
private:
// Publisher to the robot's velocity command topic
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
rclcpp::TimerBase::SharedPtr timer_;
size_t count_;
//Subscriber to robot’s odometry topic
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserScan_;
};
void Stopper::moveForward(double forwardSpeed){
auto msg = geometry_msgs::msg::Twist();
msg.linear.x = forwardSpeed;
publisher_->publish(msg);
}
void Stopper::moveStop(){
auto msg = geometry_msgs::msg::Twist();
msg.linear.x = FORWARD_SPEED_STOP;
publisher_->publish(msg);
}
void Stopper::moveRight(double turn_right_speed){
auto msg = geometry_msgs::msg::Twist();
msg.angular.z = turn_right_speed;
publisher_->publish(msg);
}
void Stopper::moveForwardRight(double forwardSpeed, double turn_right_speed){
auto msg = geometry_msgs::msg::Twist();
msg.linear.x = forwardSpeed;
msg.angular.z = turn_right_speed;
publisher_->publish(msg);
}
double odom_landmark3=1.20, odom_landmark4=1.80, odom_landmark5=2.22;
void Stopper::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg){
PositionX = odomMsg->pose.pose.position.x + homeX;
PositionY = odomMsg->pose.pose.position.y + homeY;

RCLCPP_INFO(this->get_logger(),"RobotPostion: %.2f , %.2f",PositionX, PositionY );
RCLCPP_INFO(this->get_logger(), "Robot stage: %d ", stage );
if (PositionY < odom_landmark1 && PositionX < odom_landmark1a){
stage = 1;
moveForward(FORWARD_SPEED_MIDDLE);}
else if (PositionX < odom_landmark2){
stage =2;
moveForwardRight(FORWARD_SPEED_MIDDLE,TURN_RIGHT_SPEED_MIDDLE);}
else if (PositionX < odom_landmark3){
stage = 3;
moveForward(FORWARD_SPEED_HIGH);
}
else if (PositionX < odom_landmark4){
stage = 4;
moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
}
else if (PositionX < odom_landmark5){
stage = 5;
moveForward(FORWARD_SPEED_MIDDLE);
}
else{
stage = 6;
moveStop();}
robVelocity = odomMsg->twist.twist.linear.x;
odomVelFile << numberOfCycle++ << " " << robVelocity << endl;
odomTrajFile<< PositionX <<" "<< PositionY<<endl;
}
void Stopper::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
leftRange = scan->ranges[300]; // get a range reading at the left angle
mleftRange = scan->ranges[250]; // get a range reading at the front-left angle
frontRange = scan->ranges[200]; // get a range reading at the front angle
mrightRange = scan->ranges[150]; // get a range reading at the front-right angle
rightRange = scan->ranges[100]; // get the range reading at the right angle
laserFile << leftRange << ","<< mleftRange << "," << frontRange<<"," << mrightRange << "," << rightRange <<"," <<laser_index++<< endl;
}
void Stopper::startMoving(){
odomVelFile.open("/ufs/servd01/users/sw19204/M-Drive/ros_workspace/src/tutorial_pkg/odomVelData.csv",ios::trunc);
odomTrajFile.open("/ufs/servd01/users/sw19204/M-Drive/ros_workspace/src/tutorial_pkg/odomTrajData.csv",ios::trunc); //note you should modify hhu to your username
laserFile.open("/ufs/servd01/users/sw19204/M-Drive/ros_workspace/src/tutorial_pkg/laserData.csv",ios::trunc);

while (rclcpp::ok()){ // keep spinning loop until user presses Ctrl+C
RCLCPP_INFO(this->get_logger(), "Start moving");
rclcpp::WallRate loop_rate(20);
while (rclcpp::ok()){
auto node = std::make_shared<Stopper>();
rclcpp::spin(node); // update
// RCLCPP_INFO(this->get_logger(), "Robot speed: %f",FORWARD_SPEED_LOW );
loop_rate.sleep(); // wait delta time
}
odomVelFile.close();
odomTrajFile.close();
laserFile.close();
}
}
int main(int argc, char* argv[]){
rclcpp::init(argc, argv);
Stopper stopper;
stopper.startMoving();
return 0;
}
