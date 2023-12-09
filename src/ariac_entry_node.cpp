#include <vector>
#include <queue>
#include <map>
#include <string>
#include <iostream>
#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "ur_kinematics/ur_kin.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "std_srvs/Trigger.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/VacuumGripperControl.h"
#include "osrf_gear/VacuumGripperState.h"
#include "osrf_gear/SubmitShipment.h"
#include "osrf_gear/AGVControl.h"

int count1; // Global cnt for trajectory
int count2; // Global cnt for delivered parts
bool checker;
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* tc; //Trajectory
ros::ServiceClient gclient;// gripper client
std::queue <osrf_gear::Order> all_orders; // a queue for orders
struct get_pose { geometry_msgs::Pose found_pose; bool found; std::string bin;}; // TODO: Set found to to ture if Item found
std::map <std::string, std::vector<osrf_gear::Model>> all_kits; // map for all the kits
sensor_msgs::JointState joint_states; // joints of the current state
std::map<std::string, double> base_of_bin; // Locations of bin

// Declaration for all Callback Functions
void callbk_joint(const sensor_msgs::JointState::ConstPtr &msg);
void callbk_order(const osrf_gear::Order::ConstPtr &msg);
void callbk_camera(const osrf_gear::LogicalCameraImage::ConstPtr &msg, const std::string camera);
void callbk_grip(const osrf_gear::VacuumGripperState::ConstPtr& msg);
int find_optimal(double possible_sol[8][6], bool branch_flag);
void set_orientation(geometry_msgs::Pose& the_pose);
double actuator_position(double position, std::string product_location);
void switch_grip_status(bool status);
get_pose trace_first_kit(ros::NodeHandle& nhandle, const std::string& product_type);
trajectory_msgs::JointTrajectory get_trajectory_for_foundation(const geometry_msgs::Pose& cpose,
                                                               const std::string& location, bool stay);
trajectory_msgs::JointTrajectory get_trajectory_for_arm(const geometry_msgs::Pose& desired, const double height,
                                                        const double time, bool get_kit);
void start_trajectory(trajectory_msgs::JointTrajectory& trajectory,
                      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& client, bool force_move);
void process(ros::NodeHandle& nhandle, const osrf_gear::Order &order, tf2_ros::Buffer &TFbuffer);

// callback functino for joint states
void callbk_joint(const sensor_msgs::JointState::ConstPtr &msg) {
    joint_states = *msg;
}

// callback function for orders
void callbk_order(const osrf_gear::Order::ConstPtr &msg) {
    ROS_INFO("%s Received!", msg->order_id.c_str());
    all_orders.push(*msg);
}

// callback function for kits
void callbk_camera(const osrf_gear::LogicalCameraImage::ConstPtr &msg, const std::string camera) {
    all_kits[camera] = msg->models;
}

// callback functino for checker
void callbk_grip(const osrf_gear::VacuumGripperState::ConstPtr& msg){
    checker = msg->attached;
}

// Find the optimizer for the path
int find_optimal(double possible_sol[8][6], bool branch_flag){
    if (branch_flag == true) {
        ROS_INFO("Possible solution found");
        for(int i=0;i<8;i++){
            if(possible_sol[i][1] > 3.14 && possible_sol[i][1] < 6.28 && possible_sol[i][3] > 1.57 && possible_sol[i][3] < 4.71){
                return i;
            }
        }
        return -1;
    }

    else {
        ROS_INFO("Possible solution found");
        double pi = 3.14;
        for (int i = 0; i < 8; i++) {
            double shoulder_pan = possible_sol[i][0];
            double shoulder_lift = possible_sol[i][1];
            double elbow = possible_sol[i][2];
            double wrist1 = possible_sol[i][3];
            double wrist2 = possible_sol[i][4];
            double wrist3 = possible_sol[i][5];
            bool valid_s = false;

            if (shoulder_pan < pi / 2 || shoulder_pan > 3 * pi / 2) {
                if (shoulder_lift < 3 * pi / 2 && elbow > pi) {
                    if (wrist2 > pi) {
                        valid_s = true;
                    }
                }
            } else {
                if (shoulder_lift > 3 * pi / 2 && elbow < pi) {
                    if (wrist2 > pi) {
                        valid_s = true;
                    }
                }
            }
            if (valid_s) {
                return i;
            }
        }
        return -1;
    }
}

// Set to orientation
void set_orientation(geometry_msgs::Pose& the_pose){
    the_pose.orientation.w = 0.707;
    the_pose.orientation.x = 0.0;
    the_pose.orientation.y = 0.707;
    the_pose.orientation.z = 0.0;
}

// Get the position of actuator
double actuator_position(double position, std::string product_location) {
    if(product_location == "agv1" || product_location == "agv2"){
        return base_of_bin[product_location];
    }
    double offset=0.7;
    if(position>0){
        return std::min(base_of_bin[product_location] + offset, 2.2);
    }else{
        return std::max(base_of_bin[product_location] - offset, -2.2);
    }
}

// turn the status of the gripper
void switch_grip_status(bool status){
    osrf_gear::VacuumGripperControl srv;
    srv.request.enable = status;

    if(gclient.call(srv)) {
        while(!srv.response.success && ros::ok()) {
            gclient.call(srv);
        }
        if(status){
            ROS_INFO("Vacuum Enabled");
        }else{
            ROS_INFO("Vacuum Disabled");
        }
    }
    else {
        ROS_ERROR("Error: Vacuum Failure");
    }
}

// Find the item to pickup when locating this exate OCR bin
get_pose trace_first_kit(ros::NodeHandle& nhandle, const std::string& product_type) {
    get_pose kit;
    kit.found = false;
    ros::ServiceClient material_location_client = nhandle.serviceClient<osrf_gear::GetMaterialLocations>(
            "/ariac/material_locations");
    // Establish the connection
    if (!ros::service::waitForService("/ariac/material_locations", ros::Duration(30.0))) {
        ROS_ERROR("Error: Cannot find parts location");
    }
    osrf_gear::GetMaterialLocations product_srv;
    product_srv.request.material_type = product_type;
    if (material_location_client.call(product_srv)) {
        for (const auto &unit: product_srv.response.storage_units) {
            ROS_INFO("Found %s in %s!", product_type.c_str(), unit.unit_id.c_str());
            if (unit.unit_id != "belt") {
                // continuously reading from camera
                while(all_kits[unit.unit_id].empty() && ros::ok()){
                    ros::spinOnce();
                }

                for (const auto &model: all_kits[unit.unit_id]) {
                    if (model.type == product_type) {
                        kit.found = true;
                        kit.bin = unit.unit_id;
                        kit.found_pose = model.pose;
                        break;
                    }
                }
                break;
            }
        }
    }
    return kit;
}

// Arm operatino by Action Lib
void start_trajectory(trajectory_msgs::JointTrajectory& trajectory, actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& client, bool force_move){
    control_msgs::FollowJointTrajectoryAction joint_trajectory;
    joint_trajectory.action_goal.goal.trajectory = trajectory;
    actionlib::SimpleClientGoalState act_state = client.sendGoalAndWait(joint_trajectory.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
    ROS_INFO("Action Server finished with [%i] %s", act_state.state_, act_state.toString().c_str());
    if(act_state.state_ != 6 && force_move){
        ros::Duration(0.5).sleep();
        trajectory.header.seq = count1++;
        trajectory.header.stamp = ros::Time::now();
        start_trajectory(trajectory, client, force_move);
    }
    ros::Duration(0.5).sleep();
}

// Foundation
trajectory_msgs::JointTrajectory get_trajectory_for_foundation(const geometry_msgs::Pose& cpose, const std::string& location, bool stay) {
    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.header.seq = count1++;
    joint_trajectory.header.stamp = ros::Time::now();
    joint_trajectory.header.frame_id = "/world";

    // properly set the name of the joints
    joint_trajectory.joint_names.clear();
    joint_trajectory.joint_names.emplace_back("linear_arm_actuator_joint");
    joint_trajectory.joint_names.emplace_back("shoulder_pan_joint");
    joint_trajectory.joint_names.emplace_back("shoulder_lift_joint");
    joint_trajectory.joint_names.emplace_back("elbow_joint");
    joint_trajectory.joint_names.emplace_back("wrist_1_joint");
    joint_trajectory.joint_names.emplace_back("wrist_2_joint");
    joint_trajectory.joint_names.emplace_back("wrist_3_joint");
    joint_trajectory.points.resize(1);
    joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
    joint_trajectory.points[0].positions[1] = 3.14;
    joint_trajectory.points[0].positions[2] = 3.14;
    joint_trajectory.points[0].positions[3] = 2.14;
    joint_trajectory.points[0].positions[4] = 3.27;
    joint_trajectory.points[0].positions[5] = 3.14;
    joint_trajectory.points[0].positions[6] = 0.0;
    if(stay){
        joint_trajectory.points[0].positions[0] = joint_states.position[1];
        joint_trajectory.points[0].time_from_start = ros::Duration(2.5);
    }else{
        joint_trajectory.points[0].positions[0] = actuator_position(cpose.position.y, location);
        double time = std::abs(joint_states.position[1] - joint_trajectory.points[0].positions[0]) / 1.0 + 0.1;
        joint_trajectory.points[0].time_from_start = ros::Duration(time);
    }
    return joint_trajectory;
}

// Arm anbang chen
trajectory_msgs::JointTrajectory get_trajectory_for_arm(const geometry_msgs::Pose& desired, const double height, const double time, bool get_kit) {
    // Instantiate variables to hold solutions
    double T_des[4][4];
    double q_des[8][6];

    T_des[0][3] = desired.position.x;
    T_des[1][3] = desired.position.y;
    T_des[2][3] = desired.position.z + height; // above part
    T_des[3][3] = 1.0;
    T_des[0][0] = 0.0;
    T_des[0][1] = -1.0;
    T_des[0][2] = 0.0;
    T_des[1][0] = 0.0;
    T_des[1][1] = 0.0;
    T_des[1][2] = 1.0;
    T_des[2][0] = -1.0;
    T_des[2][1] = 0.0;
    T_des[2][2] = 0.0;
    T_des[3][0] = 0.0;
    T_des[3][1] = 0.0;
    T_des[3][2] = 0.0;
    int num_sols = ur_kinematics::inverse((double *) &T_des, (double *) &q_des, 0.0);
    int q_des_indx = 0;

    // Find optimizer
    if (!get_kit) {
        q_des_indx = find_optimal(q_des, true);
    }
    else {
        q_des_indx = find_optimal(q_des, false);
    }

    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.header.seq = count1++; // assigned a seq. #
    joint_trajectory.header.stamp = ros::Time::now(); // message created
    joint_trajectory.header.frame_id = "/world"; // Frame
    joint_trajectory.joint_names.clear();
    joint_trajectory.joint_names.emplace_back("linear_arm_actuator_joint");
    joint_trajectory.joint_names.emplace_back("shoulder_pan_joint");
    joint_trajectory.joint_names.emplace_back("shoulder_lift_joint");
    joint_trajectory.joint_names.emplace_back("elbow_joint");
    joint_trajectory.joint_names.emplace_back("wrist_1_joint");
    joint_trajectory.joint_names.emplace_back("wrist_2_joint");
    joint_trajectory.joint_names.emplace_back("wrist_3_joint");
    joint_trajectory.points.resize(1);
    joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size()); // End points for movements
    for (int indy = 0; indy < 6; indy++) {
        joint_trajectory.points[0].positions[indy + 1] = q_des[q_des_indx][indy];
    }
    joint_trajectory.points[0].positions[0] = joint_states.position[1];
    joint_trajectory.points[0].time_from_start = ros::Duration(time);
    ROS_INFO("GET TRAJECTORY %i", num_sols);
    return joint_trajectory;
}

// The logical implementation for the parts distribution process
void process(ros::NodeHandle& nhandle, const osrf_gear::Order &order, tf2_ros::Buffer &TFbuffer) {
    // Connect publisher
    ros::Publisher trajectory_pub = nhandle.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm1/arm/command", 10);

    // Handle shipment
    for (const auto &shipment: order.shipments) {
        ROS_INFO("shipment type: %s", shipment.shipment_type.c_str());
        ROS_INFO("agv_id: %s", shipment.agv_id.c_str());

        std::string agv_name;
        if(shipment.agv_id == "any"){
            agv_name = "agv1";
        }else{
            agv_name = shipment.agv_id;
        }
        for (const auto &product: shipment.products) {
            ROS_INFO("type: %s", product.type.c_str());
            get_pose kit = trace_first_kit(nhandle, product.type);

            if (kit.found && count2 <= 1) {
                count2 += 1;
                auto move_base = get_trajectory_for_foundation(kit.found_pose, kit.bin, false); // move actuator
                start_trajectory(move_base, *tc, true);
                geometry_msgs::TransformStamped transformStamped; // get pose
                try {
                    transformStamped = TFbuffer.lookupTransform("arm1_base_link", "logical_camera_"+kit.bin+"_frame", ros::Time(0.0), ros::Duration(1.0));
                } catch (tf2::TransformException &ex) {
                    ROS_ERROR("%s", ex.what());
                }
                geometry_msgs::PoseStamped part_pose, goal_pose;

                part_pose.pose = kit.found_pose; // get pose
                tf2::doTransform(part_pose, goal_pose, transformStamped);
                set_orientation(goal_pose.pose);
                auto stay_above = get_trajectory_for_arm(goal_pose.pose, 0.13, 1.5, true); // find trajectory: arm
                start_trajectory(stay_above, *tc, true);
                // Start grab process, turn on vacuum
                switch_grip_status(true);
                auto goal_trajectory = get_trajectory_for_arm(goal_pose.pose, 0.015, 0.3, true);
                start_trajectory(goal_trajectory, *tc, false);
                while(!checker && ros::ok()){ // Keep trying in case the vacuum fail, them return to position
                    auto back_above_trajectory = get_trajectory_for_arm(goal_pose.pose, 0.13, 0.2, true);
                    start_trajectory(back_above_trajectory, *tc, true);
                    goal_trajectory.header.seq = count1++;
                    goal_trajectory.header.stamp = ros::Time::now();
                    start_trajectory(goal_trajectory, *tc, false);
                }
                auto default_pose = get_trajectory_for_foundation(goal_pose.pose, kit.bin, true);
                start_trajectory(default_pose, *tc, true);
                // After picking up the assigned part, the next step is to put it into the correct location
                move_base = get_trajectory_for_foundation(kit.found_pose, agv_name, false);
                start_trajectory(move_base, *tc, true);
                std::string agv;
                if(agv_name=="agv1"){
                    agv="kit_tray_1";
                }else{
                    agv="kit_tray_2";
                }
                // get pose
                try {
                    transformStamped = TFbuffer.lookupTransform("arm1_base_link", agv, ros::Time(0.0), ros::Duration(1.0));
                } catch (tf2::TransformException &ex) {
                    ROS_ERROR("%s", ex.what());
                }
                geometry_msgs::PoseStamped new_part_pose, new_goal_pose; // get pose from OCR camera
                new_part_pose.pose = product.pose;
                tf2::doTransform(new_part_pose, new_goal_pose, transformStamped);
                set_orientation(new_goal_pose.pose);
                auto first_pose = get_trajectory_for_arm(new_goal_pose.pose, 0.1, 2.5, false);
                if(agv_name=="agv1"){
                    first_pose.points[0].positions[1] = 2.1;
                }else{
                    first_pose.points[0].positions[1] = 4.18;
                }
                start_trajectory(first_pose, *tc, true);
                goal_trajectory = get_trajectory_for_arm(new_goal_pose.pose, 0.1, 1.0, false);
                start_trajectory(goal_trajectory, *tc, true);
                switch_grip_status(false); // gripper off
            }
        }
        ros::Duration(1.0).sleep();
        std::string agv_id=agv_name;
        std::string shipment_name=shipment.shipment_type;
        ros::ServiceClient submit_client;
        if (agv_id == "agv1") {
            submit_client = nhandle.serviceClient<osrf_gear::AGVControl>("ariac/agv1");
        }
        else if (agv_id == "agv2") {
            submit_client = nhandle.serviceClient<osrf_gear::AGVControl>("ariac/agv2");
        }
        else {
            ROS_WARN("Invalid AGV ID");
            return;
        }
        osrf_gear::AGVControl submit_srv;
        submit_srv.request.shipment_type = shipment_name;
        if (submit_client.call(submit_srv)) {
            while (!submit_srv.response.success) {
                submit_client.call(submit_srv);
            }
            ROS_INFO("Submitted Shipment %s on %s", shipment_name.c_str(), agv_id.c_str());
            ROS_INFO("Submitted Message : %s", submit_srv.response.message.c_str());
        }
        else {
            ROS_WARN("Fail to get submission service");
        }
    }
}

int main(int argc, char **argv) {
    // INIT process
    count1 = 0;
    ros::init(argc, argv, "ariac_entry_node");
    ros::NodeHandle nhandle;
    std_srvs::Trigger begin_comp;
    ros::ServiceClient begin_client = nhandle.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    base_of_bin = { // Locations
            {"bin4", 0.383},
            {"bin5", 1.15},
            {"bin6", 1.916},
            {"agv1", 2.2},
            {"agv2", -2.2}
    };
    bool service_call_succeeded;
    service_call_succeeded = begin_client.call(begin_comp);
    // Ensure the status of the competition
    if (service_call_succeeded) {
        if (begin_comp.response.success) {
            ROS_INFO("Start Success: %s", begin_comp.response.message.c_str());
        } else {
            ROS_WARN("Start Failure: %s", begin_comp.response.message.c_str());
        }
    } else {
        ROS_ERROR("Competition service call failed!");
        ros::shutdown();
    }
    // Action Client
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> traj_ac("/ariac/arm1/arm/follow_joint_trajectory", true);
    traj_ac.waitForServer();
    ROS_INFO("Successfully connected to action server!");
    tc = &traj_ac;
    gclient = nhandle.serviceClient<osrf_gear::VacuumGripperControl>("ariac/arm1/gripper/control"); // Gripper
    all_orders={};
    ros::Subscriber sub_orders = nhandle.subscribe("/ariac/orders", 10, callbk_order); // Order management
    ros::Subscriber joint_states_h = nhandle.subscribe("/ariac/arm1/joint_states", 10, callbk_joint); // Status of joints
    ros::Subscriber sub_vacuum = nhandle.subscribe("/ariac/arm1/gripper/state", 10, callbk_grip); // Vacuum gripper
    ros::AsyncSpinner spinner(3); // start 3 threads
    spinner.start();
    // Init camera
    ros::Subscriber agv_subscriber1 = nhandle.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_agv1", 10,
                                                                                       boost::bind(callbk_camera, _1, "agv1"));
    ros::Subscriber agv_subscriber2 = nhandle.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_agv2", 10,
                                                                                       boost::bind(callbk_camera, _1, "agv2"));
    ros::Subscriber bin_subscriber1 = nhandle.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin1", 10,
                                                                                       boost::bind(callbk_camera, _1, "bin1"));
    ros::Subscriber bin_subscriber2 = nhandle.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin2", 10,
                                                                                       boost::bind(callbk_camera, _1, "bin2"));
    ros::Subscriber bin_subscriber3 = nhandle.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin3", 10,
                                                                                       boost::bind(callbk_camera, _1, "bin3"));
    ros::Subscriber bin_subscriber4 = nhandle.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin4", 10,
                                                                                       boost::bind(callbk_camera, _1, "bin4"));
    ros::Subscriber bin_subscriber5 = nhandle.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin5", 10,
                                                                                       boost::bind(callbk_camera, _1, "bin5"));
    ros::Subscriber bin_subscriber6 = nhandle.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin6", 10,
                                                                                       boost::bind(callbk_camera, _1, "bin6"));
    tf2_ros::Buffer TFbuffer; // Transform buffer
    tf2_ros::TransformListener tfListener(TFbuffer); // Handle buffer update
    ros::Rate rate(10); // Freq
    while (ros::ok()) { // Game Start
        ros::spinOnce();
        if (all_orders.empty()) {
            ROS_WARN("Wait for order.");
            ros::spinOnce();
            rate.sleep();
            continue;
        }
        process(nhandle, all_orders.front(), TFbuffer);
        all_orders.pop();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

