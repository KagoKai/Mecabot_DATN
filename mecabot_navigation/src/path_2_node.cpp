#define NUM_OF_GOALS    1
#define PI              3.141593

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

typedef struct
{
    float x;        // Goal X position (m)
    float y;        // Goal Y position (m)
    float theta;    // Goal Yaw angle (rad)
}Position_t;
const Position_t goal_pos[] =
{
    { .x =  3.6f, .y =  0.0f, .theta =  0.0f }, // Stop Point 1
};
static move_base_msgs::MoveBaseGoal goal;
static bool start_flag = false;

void startSignalCallback(const std_msgs::Bool& start_signal_msg)
{
    if (start_signal_msg.data == true)
    {
        ROS_INFO("Start signal received. Initialize demo path ...");
        start_flag = true;
        ros::Duration(5.0).sleep();
    }
}

void updateGoalMsg(int id = 0)
{
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = goal_pos[id].x;
    goal.target_pose.pose.position.y = goal_pos[id].y;
    goal.target_pose.pose.position.z = 0;

    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goal_pos[id].theta);
}

int main(int argc, char** argv)
{
    // Initialize the ROS system
    ros::init(argc, argv, "path_node");

    // Initialize Node Handle
    ros::NodeHandle nh;

    // Loop at 50Hz
    ros::Rate loop_rate(20);

    // Subscriber to start signal
    ros::Subscriber sub_start_signal = nh.subscribe("start_signal", 100, startSignalCallback);
    // Publisher to cmd_vel

    // Tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);
    //wait for the action server to come up
    ROS_INFO("Waiting for the move_base action server to come up");
    while(!ac.waitForServer())
    {
        ROS_INFO("...");
    }
    ROS_INFO("Connected to the move_base action server.");

    while (ros::ok())
    {
        if (start_flag)
        {
            for (int i = 0; i < NUM_OF_GOALS; i++)
            {
                ROS_INFO("Sending goal. Going to position %d", i+1);
                ROS_INFO("X: %.2f", goal_pos[i].x);
                ROS_INFO("Y: %.2f", goal_pos[i].y);
                ROS_INFO("Theta: %.2f", goal_pos[i].theta);
                updateGoalMsg(i);
                ac.sendGoal(goal);
                ac.waitForResult();

                if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("Goal reached !");
                }

                ros::Duration(2.0).sleep(); // Stay in place for 5 seconds
                
                loop_rate.sleep();
            }

            ROS_INFO("The demo path program is finished !");
            start_flag = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}