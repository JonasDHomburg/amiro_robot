#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf2/LinearMath/Quaternion.h"
#include "std_msgs/String.h"
#include <ros/ros.h>
#include "unistd.h"
#include "math.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>

std::vector<geometry_msgs::PoseStamped> goals;
std::vector<geometry_msgs::PoseStamped> goalRoute;
ros::Subscriber goal_reached_sub;
ros::Subscriber base_link;
ros::Publisher chatter_pub;
static tf::TransformListener* tfListener = NULL;
bool init_stack_goals = false;
bool has_reached = false;
bool circle = false;
int subscriber_count = 0;
int current_status = 0;

//Parameter Server Variables
bool stack_goals;
int goal_threshold;
int number_circles;
bool clockwise_dir;
int samples;
double radius;
double threshold;

//Navigation in Circles -> First position is the mid of the circle to be driven
auto circleRoute(geometry_msgs::PoseStamped start_pos) {
  std::vector<geometry_msgs::PoseStamped> circleGoalRoutes;
  geometry_msgs::PoseStamped goal;
  double clockwise = (clockwise_dir == true)?-1:1;

  tf2::Quaternion quat_start_pos;
  tf2::fromMsg(start_pos.pose.orientation, quat_start_pos); //= start_pos.pose.orientation;
  double rad_start_pos = (start_pos.pose.orientation.z > 0)?quat_start_pos.getAngle():-quat_start_pos.getAngle();
  double rad_per_sample = 2*M_PI/samples;

  //Adding Goals
  goal.header.frame_id = start_pos.header.frame_id;
  int loop_samples = number_circles>1?samples+1:samples;
  //circleGoalRoutes.push_back(start_pos);
  for (int i = 0; i < number_circles; i++) {
    for (int j=0; j < loop_samples; j++) { 
      //Translation
      //goal.header.stamp = ros::Time::now();
      goal.header.stamp = start_pos.header.stamp;
      goal.pose.position.x = radius*cos(clockwise*j*rad_per_sample+rad_start_pos)+start_pos.pose.position.x;
      goal.pose.position.y = radius*sin(clockwise*j*rad_per_sample+rad_start_pos)+start_pos.pose.position.y;
      goal.pose.position.z = start_pos.pose.position.z;

      //Rotation 
      tf2::Quaternion quaternion;
      quaternion.setRPY(0, 0, clockwise*j*rad_per_sample+rad_start_pos+clockwise*M_PI/2);
      goal.pose.orientation = tf2::toMsg(quaternion);
    
      //Collect Goal
      circleGoalRoutes.push_back(goal);
    }
    if (i==number_circles-2){
      loop_samples += 1;
    }
  }
  return circleGoalRoutes;
}


/* auto goalInProximity(auto position, auto goal) {
  goal_bbox = {goals.front().pose.position.x + }
} */

/* //Legacy approach
void samplePoint(geometry_msgs::PoseStamped goal1, geometry_msgs::PoseStamped goal2, int samples) {
  geometry_msgs::PoseStamped new_goal = goal2;
  std::vector<int> range_vector = pyRange(samples+1);
  for (int i: range_vector) {
    auto g1_x = goal1.pose.position.x;
    auto g1_y = goal1.pose.position.y;
    auto g2_x = goal2.pose.position.x;
    auto g2_y = goal2.pose.position.y;
    auto g_s_x = ((samples-i)/samples)*g1_x+(i/samples)*g2_x;
    auto g_s_y = ((samples-i)/samples)*g1_y+(i/samples)*g2_y;
    new_goal.pose.position.x = g_s_x;
    new_goal.pose.position.x = g_s_y;
    printf("\n Range_vector: %d", i);
    goalRoute.push_back(new_goal);
  }
} */

/* //Legacy approach
// ToDo: Schnittstellen werden doppelt als Goal angegeben
void createGoalRoute(std::vector<geometry_msgs::PoseStamped> goal, int samples) {
  std::vector<int> range_vector = pyRange(goal.size()-1);
  ROS_INFO("\n Post_PyRange");
  for (int i: range_vector) {
    ROS_INFO("\n Create Goal: %d", i);
    samplePoint(goal.at(i), goal.at(i+1), samples);
    ROS_INFO("\n Finished all Goals");
  }
} */

/* void incSubscriberCount() {
  subscriber_count++;
  if(circle) {
    goals = circleRoute(goals.front());
    //circle = false;
  }
  else if (subscriber_count >= goal_threshold && init_stack_goals) {
    ROS_INFO("Pre_CreateGoal");
    //createGoalRoute(goals, 10);
    ROS_INFO("Post_CreateGoal");
    goals = goalRoute;
    goalRoute.clear();
    printf("Goal Size: %lu", goals.size());
    stack_goals = false;
  }
  //ROS_INFO("Subcount: [%d]", subscriber_count);
} */

void updateGoals() {
  
    if(goals.size() > 0) {
      goals.erase(goals.begin());
    }
    if(goals.size() > 0) {
      chatter_pub.publish(goals.front());
    }
    
}

void chatterCallbackGoals(const geometry_msgs::PoseStamped msg) {
  goals.push_back(msg);
  if(circle) {
    goals = circleRoute(goals.front());
    //circle = false;
  }
  else if (goals.size() >= goal_threshold && init_stack_goals) {
    stack_goals = false;
  }
  //incSubscriberCount();
}

void chatterCallbackGoalsReached(const actionlib_msgs::GoalStatusArray msg) {
  if(stack_goals && !circle) {
    //ROS_INFO("Stacking Goals");
  }
  else if(!msg.status_list.empty() && msg.status_list.size() > 1) {
    //ROS_INFO("Status: %d", msg.status_list.at(1).status);
    switch(msg.status_list.at(1).status) { //Goal reached
      case 1:
        current_status = 1;
        has_reached=false;
        //ROS_INFO("Goal active");
        break;
      case 2:
        current_status = 2;
        has_reached=false;
        //ROS_INFO("Goal overwritten");
        break;
      case 3:
        current_status = 3;
        if(!has_reached) {
          updateGoals();
          has_reached=true;
        }
        if(goals.size() <= 0) {
          if(init_stack_goals) {
            stack_goals = true;
          }
          else if(circle) {
            //circle = true;
          }
        }
        //ROS_INFO("Reached and Goal number: %lu", goals.size());
        break;
      default:
        ROS_INFO("Idle");
    }
  }
  else {
    if(goals.size() > 0)
      chatter_pub.publish(goals.front());
  }
}

void chatterCallbackBaselink(const nav_msgs::Odometry msg) {
  /* switch(current_status) { //Goal reached
    case 1: */
    if (goals.size()>0){
      double x, y, euclidean_distance;
      geometry_msgs::PoseStamped original_pose;
      original_pose.pose = msg.pose.pose;
      original_pose.header = msg.header;
      geometry_msgs::PoseStamped pose_transformed;
      tfListener->waitForTransform(msg.header.frame_id, "map", msg.header.stamp, ros::Duration(0.5));
      tfListener->transformPose("map", original_pose, pose_transformed);
      ROS_INFO("goal: x: %f, y: %f", goals.front().pose.position.x, goals.front().pose.position.y);
      ROS_INFO("pose: x: %f, y: %f", pose_transformed.pose.position.x, pose_transformed.pose.position.y);
      x = goals.front().pose.position.x - pose_transformed.pose.position.x;
      y = goals.front().pose.position.y - pose_transformed.pose.position.y;
      euclidean_distance = pow(x, 2) + pow(y, 2);
      euclidean_distance = sqrt(euclidean_distance);
      ROS_INFO("eudclidean distance: %f", euclidean_distance);
      if(euclidean_distance < threshold) {
        updateGoals();
      }
    }
    /*   break;
    case 2:
      break;
    case 3:
      break;
    default:
      ROS_INFO("Idle");
  } */
}

int main(int argc, char **argv) {
  ros::init(argc, argv, ros::this_node::getName());
  ros::NodeHandle node("~");
  tfListener = new(tf::TransformListener);

  node.param<bool>("stack_goals_first", stack_goals, false);
  init_stack_goals = stack_goals;
  node.param<int>("number_of_goals", goal_threshold, 3);
  node.param<bool>("drive_circle", circle, false);
  node.param<int>("number_circles", number_circles, 3);
  node.param<double>("radius_of_circles", radius, 1.0);
  node.param<int>("samples_per_circle", samples, 4);
  node.param<bool>("clockwise_direction", clockwise_dir, true);
  node.param<double>("threshold", threshold, 0.1);

  ROS_INFO("[%s] stack_goals_first: %i", ros::this_node::getName().c_str(), stack_goals);
  ROS_INFO("[%s] number_of_goals: %d", ros::this_node::getName().c_str(), goal_threshold);
  ROS_INFO("[%s] drive_circle: %i", ros::this_node::getName().c_str(), circle);
  ROS_INFO("[%s] number_circles: %d", ros::this_node::getName().c_str(), number_circles);
  ROS_INFO("[%s] radius_of_circles: %f", ros::this_node::getName().c_str(), radius);
  ROS_INFO("[%s] samples_per_circle: %d", ros::this_node::getName().c_str(), samples);
  ROS_INFO("[%s] clockwise_direction: %i", ros::this_node::getName().c_str(), clockwise_dir);
  ROS_INFO("[%s] threshold: %f", ros::this_node::getName().c_str(), threshold);

  ros::Subscriber sub = node.subscribe("/move_base_simple/goal", 1000, chatterCallbackGoals);
  chatter_pub = node.advertise<geometry_msgs::PoseStamped>("/stacked_move_base_simple/goal", 1);
  goal_reached_sub = node.subscribe("/move_base/status", 1000, chatterCallbackGoalsReached);
  base_link = node.subscribe("/amiro7/odom", 1000, chatterCallbackBaselink);

  ros::spin();

  return 0;
}