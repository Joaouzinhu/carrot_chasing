#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include <cmath>

geometry_msgs::Point robo;
geometry_msgs::Point ponto_inicial;
geometry_msgs::Point ponto_final;
geometry_msgs::Point ponto_C;
geometry_msgs::Twist vel_robo;
geometry_msgs::Quaternion qt;

ros::Publisher pub_vel;

double delta = 0.5, ang_yaw, theta, var_psi, distance_R,distance_RU,beta, theta_U,psi,k_PA;
void odom_callback(const nav_msgs::OdometryConstPtr &odom_msg)
{
    ponto_inicial.x = 4;
    ponto_inicial.y = 1.5;
    ponto_final.x = -3;
    ponto_final.y = 1.5;
    robo.x = odom_msg->pose.pose.position.x;
    robo.y = odom_msg->pose.pose.position.y;
    qt = odom_msg->pose.pose.orientation;
    ang_yaw = tf::getYaw(qt)*180/M_PI;
    theta = atan2(ponto_final.y-ponto_inicial.y,ponto_final.x-ponto_inicial.x)*180/M_PI;
    theta_U = atan2(robo.y-ponto_inicial.y,robo.x-ponto_inicial.x)*180/M_PI;
    beta = theta-theta_U;
    distance_RU = (robo.x-ponto_inicial.x)/cos(theta_U*M_PI/180);
    distance_R = distance_RU * cos(beta*M_PI/180);
    ponto_C.x = (distance_R + delta)*cos(theta*M_PI/180)+ ponto_inicial.x;
    ponto_C.y = (distance_R + delta)*sin(theta*M_PI/180)+ ponto_inicial.y;
    psi = atan2(ponto_C.y-robo.y,ponto_C.x-robo.x)*180/M_PI;
    var_psi = psi - ang_yaw;
    if(var_psi > 180)
    {
        var_psi = var_psi - 360;
    }
    if(var_psi < -180)
    {
        var_psi = var_psi + 360;
    }
    vel_robo.linear.x = 1;
    vel_robo.angular.z = 0.05*var_psi;
    if(std::abs(vel_robo.angular.z)>10)
    {
        vel_robo.angular.z = 10*(var_psi/std::abs(var_psi));
    }    
    pub_vel.publish(vel_robo);
    ROS_INFO("Velocidade : %f",vel_robo.angular.z);
        
}


int main(int argc,char** argv)
{
    ros::init(argc,argv,"Followtraj");
    ros::NodeHandle node;
    ros::Subscriber sub_odom = node.subscribe("/vrep/vehicle/odometry",1,odom_callback);
    
    pub_vel = node.advertise<geometry_msgs::Twist>("ros_odom",1);
    
    ros::spin();
    
}