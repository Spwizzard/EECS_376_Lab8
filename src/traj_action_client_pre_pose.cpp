// traj_action_client_pre_pose: 
// uses right and left arm trajectory action servers to send robot to a hard-coded pre pose

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_trajectory_streamer/baxter_trajectory_streamer.h>
#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/EndEffectorCommand.h>

#include<baxter_trajectory_streamer/trajAction.h>
using namespace std;
#define VECTOR_DIM 7 // e.g., a 7-dof vector

int g_done_count = 0;
int g_done_move = false;

baxter_core_msgs::EndEffectorCommand gripper_cmd_open, gripper_cmd_close;
void rightArmDoneCb(const actionlib::SimpleClientGoalState& state,
        const baxter_trajectory_streamer::trajResultConstPtr& result) {
    ROS_INFO(" rtArmDoneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d", result->return_val);
    g_done_count++;
    g_done_move = true;
}

void leftArmDoneCb(const actionlib::SimpleClientGoalState& state,
        const baxter_trajectory_streamer::trajResultConstPtr& result) {
    ROS_INFO(" leftArmDoneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d", result->return_val);
        g_done_count++;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "traj_action_client_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle        

    gripper_cmd_open.id = 65538;
    gripper_cmd_open.command ="go";
    //gripper_cmd_open.args = "{'position': 100.0}'"; //oops
    gripper_cmd_open.args = "{\"position\": 100.0}";
    gripper_cmd_open.sender = "gripper_publisher";
    gripper_cmd_open.sequence = 2;
    
    gripper_cmd_close.id = 65538;
    gripper_cmd_close.command ="go";
    //gripper_cmd_close.args = "{'position': 0.0}'"; //oops
    gripper_cmd_close.args = "{\"position\": 5.0}";
    gripper_cmd_close.sender = "gripper_publisher"; 
    gripper_cmd_close.sequence = 3;

    ros::Publisher gripper_publisher_right_ = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command", 1, true);

    Eigen::VectorXd q_chest_right_arm;
    Eigen::VectorXd q_approach_right_arm;
    Eigen::VectorXd q_gripping_right_arm;
    Eigen::VectorXd q_leave_right_arm;
    Eigen::VectorXd q_final_right_arm;
    Eigen::VectorXd q_vec_right_arm;

    std::vector<Eigen::VectorXd> down_path_right, up_path_right;
    trajectory_msgs::JointTrajectory down_trajectory_right, up_trajectory_right; // empty trajectories     
    
    //here are hard-coded joint angles for left and right arm poses
    cout << "setting pre-poses: " << endl;
    q_chest_right_arm.resize(7);
    q_approach_right_arm.resize(7);
    q_gripping_right_arm.resize(7);
    q_leave_right_arm.resize(7);
    q_final_right_arm.resize(7);
    q_chest_right_arm << 0.35051461, -0.790383601, 0.1909806081, 2.608917825, -1.8875633595, -1.2751215299, -0.1568495356;
    q_approach_right_arm << 1.0078253776, -0.8325680726, -0.1292378814, 0.8479078805, -3.0089033154, -1.5581409853, 3.058757691;
    q_gripping_right_arm << 1.2068593849, 0.0099708751, -0.3566505332,0.7876991346, -2.5820731612,-0.7205874751,3.0591411862;
    q_leave_right_arm << 1.0078253776, -0.8325680726, -0.1292378814, 0.8479078805, -3.0089033154, -1.5581409853, 3.058757691;
    q_final_right_arm << 0.35051461, -0.790383601, 0.1909806081, 2.608917825, -1.8875633595, -1.2751215299, -0.1568495356; 
    //corresponding values to mirror the left arm pose:

    ROS_INFO("instantiating a traj streamer");
    Baxter_traj_streamer baxter_traj_streamer(&nh); //instantiate a Baxter_traj_streamer object and pass in pointer to nodehandle for constructor to use  
    // warm up the joint-state callbacks; want to make sure the joint states are valid
    cout << "warming up callbacks..." << endl;
    for (int i = 0; i < 100; i++) {
        ros::spinOnce(); //the baxter_traj_streamer needs spins for its updates
        ros::Duration(0.01).sleep();
    }

    //get current pose of left and right arms:
    cout<<"right arm is at: "<<baxter_traj_streamer.get_q_vec_right_arm_Xd().transpose()<<endl;
    //cout<<"enter 1";
    //cin>>ans;    
    q_vec_right_arm = baxter_traj_streamer.get_q_vec_right_arm_Xd(); 
    cout << "right-arm current state:" << q_vec_right_arm.transpose() << endl;

    down_path_right.push_back(q_chest_right_arm);
    down_path_right.push_back(q_approach_right_arm);
    down_path_right.push_back(q_gripping_right_arm);
    

    up_path_right.push_back(q_gripping_right_arm);
    up_path_right.push_back(q_leave_right_arm);
    up_path_right.push_back(q_final_right_arm);

    cout << "stuffing traj: " << endl;
    //convert from vector of 7dof poses to trajectory messages  
    baxter_traj_streamer.stuff_trajectory_right_arm(down_path_right, down_trajectory_right);
    baxter_traj_streamer.stuff_trajectory_right_arm(up_path_right, up_trajectory_right);
    
    
    // goal objects compatible with the arm servers
    baxter_trajectory_streamer::trajGoal goal_right;

    //instantiate clients of the two arm servers:
    actionlib::SimpleActionClient<baxter_trajectory_streamer::trajAction> right_arm_action_client("rightArmTrajActionServer", true);

    int loopCount = 0;

    while(ros::ok()){

        ROS_INFO("waiting for right-arm server: ");
        bool server_exists = right_arm_action_client.waitForServer(ros::Duration(1.0));
        while (!server_exists) {
            ROS_WARN("waiting on right-arm server...");
            ros::spinOnce();
            ros::Duration(1.0).sleep();
            server_exists = right_arm_action_client.waitForServer(ros::Duration(1.0));
        }
        ROS_INFO("connected to right-arm action server"); // if here, then we connected to the server;  

        ROS_INFO("sending down goal to right arm: ");

        goal_right.trajectory = down_trajectory_right;

        right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb);

        while (g_done_move != true) { //changed count test to "1", since testing w/ right-arm only
            ROS_INFO("waiting to finish down trajectory");
            ros::Duration(0.5).sleep();
        }
        g_done_move = false;
        //ros::Duration(5).sleep();

        gripper_publisher_right_.publish(gripper_cmd_close);
        ros::Duration(0.5).sleep();

        ROS_INFO("sending up goal to right arm: ");

        goal_right.trajectory = up_trajectory_right;

        right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb);

        while (g_done_move != true) { //changed count test to "1", since testing w/ right-arm only
            ROS_INFO("waiting to finish down trajectory");
            ros::Duration(0.5).sleep();
        }
        g_done_move = false;
        //ros::Duration(5).sleep();

        ROS_INFO("sending down goal to right arm: ");

        goal_right.trajectory = down_trajectory_right;

        right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb);

        while (g_done_move != true) { //changed count test to "1", since testing w/ right-arm only
            ROS_INFO("waiting to finish down trajectory");
            ros::Duration(0.5).sleep();
        }
        g_done_move = false;
        //ros::Duration(5).sleep();

        gripper_publisher_right_.publish(gripper_cmd_open);
        ros::Duration(0.5).sleep();

        ROS_INFO("sending up goal to right arm: ");

        goal_right.trajectory = up_trajectory_right;

        right_arm_action_client.sendGoal(goal_right, &rightArmDoneCb);

        while (g_done_move != true) { //changed count test to "1", since testing w/ right-arm only
            ROS_INFO("waiting to finish down trajectory");
            ros::Duration(0.5).sleep();
        }
        g_done_move = false;
        //ros::Duration(5).sleep();

        loopCount++;
        ROS_INFO("loop count: %d", loopCount);
        ros::spinOnce();
    }

    return 0;
}

