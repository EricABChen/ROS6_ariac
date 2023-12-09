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

