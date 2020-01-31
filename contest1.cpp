#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <stdio.h>
#include <cmath>
#include <chrono>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <inttypes.h> 
#include <iostream>
#include <tuple>
#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad)*180. / M_PI)
#define DEG2RAD(deg) ((deg)*M_PI / 180.)
float final_ori = 0.0;
float final_ori_max =  0.0 ; 
float final_ori_min = 0.0 ;
bool  revolution =false; 
float angular = 0.0;
float linear = 0.0;
float posX = 0.0, posY = 0.0, yaw = 0.0;
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
uint8_t leftState = bumper[kobuki_msgs::BumperEvent::LEFT];

float minLaserDist = std::numeric_limits<float>::infinity();
int32_t nLasers = 0, desiredNLasers = 0, desiredAngle = 5;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &msg)
{
    //Access using bumper [kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    //fill with your code

    minLaserDist = std::numeric_limits<float>::infinity();
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = desiredAngle * M_PI / (180 * msg->angle_increment);
    //ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);
    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min)
    {
        for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx)
        {
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
        }
    }
    else
    {
        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx)
        {
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
        }
    }
}



void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    tf::getYaw(msg->pose.pose.orientation);
  //  ROS_INFO("Position:(%f,%f) Orientation: %f rad or %f degrees. ", posX, posY, yaw, RAD2DEG(yaw));
}

 float  sum_angles (float a, float b )
 {
     float sum= 0. ; 
  if (a + b >= M_PI) 
    {
       sum = -M_PI + (a + b - M_PI) ;
       return sum  ;
    }
    else if (a+b <= -M_PI   )
    {
       sum = M_PI  + (a+b + M_PI );
       return sum;
    }
    else
    {
    sum = a+b ; 
       return sum;
    }  
 }


 void Turn ( float angle, float *linear, float *angular , bool *revolution , float *final_ori , float *final_ori_max , float *final_ori_min)
{  
    // positive angle means turning CCW, negative angle means turning 
     if (!*revolution)
     {
     float angler =0.0; 
    angler = DEG2RAD (angle) ; 
    *final_ori = sum_angles (*final_ori , angler );
    *final_ori_max= sum_angles(*final_ori , M_PI/90.);
    *final_ori_min= sum_angles(*final_ori , -M_PI/90.) ; 
    ROS_INFO("Expected angle at %f  %f   %f", RAD2DEG (*final_ori), RAD2DEG (*final_ori_max), RAD2DEG (*final_ori_min));
    // checking for angle 
    if (angle < 0 ) 
    {
     *angular = M_PI/12. ;
    }
    else if (angle >0)
    {
     *angular = -M_PI/12. ;
    }
    else 
    {
      *angular =0.0  ;
    } 

    *linear = 0.0 ; 
    *revolution= true ;
    
    }

     else if (*revolution)
    {
              
              if (yaw <= *final_ori_max  && yaw >  *final_ori_min && *final_ori_max > *final_ori_min)
            {
              *revolution = false ;
              *angular= 0.0 ; 
              *linear= 0.0; 
              ROS_INFO("Rotation done");
              
            }
             else if (  *final_ori_max < *final_ori_min && (yaw <= *final_ori_max  ||  yaw  > *final_ori_min ))
             {
                  *revolution = false ;
                  *angular= 0.0 ; 
                  *linear= 0.0; 
                   ROS_INFO("Rotation done");
             }
              else
              {
              *angular= M_PI/12. ; 
              *linear = 0.0 ;
              ROS_INFO("Expected angle  rot %f", RAD2DEG (*final_ori));
              *revolution= true; 
              
              }

    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    

    float angular = 0.0;
    float linear = 0.0;
    revolution= false; 
    bool done = false;
    bool turn_complete= false ; 
    float final_ori = 0.0;
    float final_ori_max =  0.0 ; 
    float final_ori_min = 0.0 ;

    while (ros::ok() && secondsElapsed <= 480)
    {
        ros::spinOnce();
        
        double time = double (secondsElapsed);
        ROS_INFO("lin: %f  ang:%f  time(%f )Pos: (%f, %f) yaw: %f laser: %f",linear, angular ,time,posX, posY, RAD2DEG(yaw), minLaserDist);
          
        bool any_bumper_pressed = false;
        for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx)
         {
             any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
         }
        if ( !revolution  && !any_bumper_pressed  && !done )
        {    ROS_INFO("initializing revolution");
            angular = M_PI/6. ;
            linear = 0.0;
            if (time >=8 )
            {
             revolution=true ;
            }
        }
        else if (!done && revolution && !any_bumper_pressed && time >= 8. )
        {
            if ( yaw < (M_PI/180) && yaw >  (-M_PI/180)) 
            {       
                ROS_INFO("Revolution DONE ");    
                done =true ;
                revolution=false;  
            }
            else
            { 
              revolution=true; 
              ROS_INFO("still rotating ");   
                   
            }
        }

        else if ( done && time >= 5. )
        {
            ROS_INFO("Starting procedure");   

          if ( minLaserDist >= 0.60 && !any_bumper_pressed && !revolution)
          {
            angular = 0.0;
            linear = 0.2;
            ROS_INFO("going straight ");   
          }  
        
          else if (  (minLaserDist <= 0.60 || any_bumper_pressed || revolution ))
          {

              Turn(90. , &linear, &angular , &revolution, &final_ori , &final_ori_max , &final_ori_min);
             
          }
            
        }
        
        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
        loop_rate.sleep();
        
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
    }

    return 0;
}
