#include "std_srvs/Trigger.h"
#include <ros/ros.h>
#include "std_srvs/SetBool.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/StorageUnit.h"
#include "osrf_gear/VacuumGripperControl.h"
#include "osrf_gear/VacuumGripperState.h"
#include "osrf_gear/AGVControl.h"
#include <string>
#include "geometry_msgs/Pose.h"
#include "std_msgs/String.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include "ur_kinematics/ur_kin.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

std::vector<osrf_gear::Order> order_vector;
sensor_msgs::JointState joint_states;
ros::ServiceClient materialLocations;

std::vector<osrf_gear::LogicalCameraImage> camera_Logical_Images = std::vector<osrf_gear::LogicalCameraImage>(10);
ros::ServiceClient gml;
tf2_ros::Buffer tfBuffer;

actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *trajectory_as;

ros::Publisher trajectoryPub;

// Instantiate variables for use with the kinematic system.
double T_pose[4][4], T_des[4][4];
double q_pose[6], q_des[8][6];

static trajectory_msgs::JointTrajectory get_trajectory(geometry_msgs::Point dest);
static void action_method(trajectory_msgs::JointTrajectory joint_trajectory, double duration);

// Phase 1
ros::ServiceClient gripper_client;
osrf_gear::VacuumGripperState gripper_state;

// Phase 2
static trajectory_msgs::JointTrajectory trajectoryHelper();
static void moveArm(double pose);
static void operateGripper(bool attach, geometry_msgs::Point dest, double pose);
ros::ServiceClient agv1_client;
ros::ServiceClient agv2_client;
static void moveArmAndGripper(double pose, geometry_msgs::Point dest);
static const double THROTTLE = 30;
void operateGripperHelper(bool attach);

void sendOutAGV(osrf_gear::AGVControl agv, int agvnum)
{
    if (agvnum == 1)
    {
        agv1_client.call(agv);
    }
    else
    {
        agv2_client.call(agv);
    }
}

void orderCallback(const osrf_gear::Order msg)
{
    ROS_ERROR("Starting Order callback");
    order_vector.push_back(msg);
}

void jointCallback(const sensor_msgs::JointState msg)
{
    ROS_INFO_THROTTLE(THROTTLE, "Starting Joint callback");
    joint_states = msg;
    joint_states.header.frame_id = "not_empty";

    std::string str;

    for (std::string s : joint_states.name)
    {
        str = str + " " + s + " ";
    }
    ROS_INFO_STREAM_THROTTLE(THROTTLE, str);
}

static geometry_msgs::TransformStamped transformHelper(std::string frame)
{
    geometry_msgs::TransformStamped tfStamped;
    try
    {
        tfStamped = tfBuffer.lookupTransform("arm1_base_link", frame,
                                             ros::Time(0.0), ros::Duration(1.0));
        ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),
                  tfStamped.child_frame_id.c_str());
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    return tfStamped;
}

void printOrderModelPose()
{
    ROS_INFO_THROTTLE(THROTTLE, "Handling Arm");
    if (joint_states.header.frame_id == "uninitialized")
    {
        return;
    }

    if (order_vector.size() == 0)
    {
        return;
    }

    std::vector<osrf_gear::Shipment> shipments = order_vector.front().shipments;
    ROS_WARN_STREAM("Got shipments amount" + std::to_string(shipments.size()));

    for (osrf_gear::Shipment shipment : shipments)
    {
        double agv_lin;
        int agv_num;
        std::string agv_camera_frame;
        std::string agv_id = shipment.agv_id;
       
        double z_modify;

        if (agv_id == "agv1")
        {
            agv_lin = 2.2;  //Y value of the tray1
            agv_num = 1;
            agv_camera_frame = "logical_camera_agv1_frame";
            z_modify = 0.;
        }
        else
        {
            agv_lin = -2.25; // Y value of the tray2
            agv_num = 2;
            agv_camera_frame = "logical_camera_agv2_frame";
            z_modify = 0.25;
        }

        for (osrf_gear::Product product : shipment.products)
        {
            std::string productType = product.type;
            geometry_msgs::PoseStamped productDest;
            productDest.pose = product.pose;

            osrf_gear::GetMaterialLocations gmlService;
            gmlService.request.material_type = productType;
            gml.call(gmlService);
            for (osrf_gear::StorageUnit su : gmlService.response.storage_units)
            {
                // Ignore Parts on the belt
                if (su.unit_id == "belt")
                {
                    continue;
                }
                const char *binName = su.unit_id.c_str();
                int binNum;
                sscanf(binName, "bin%d", &binNum);
                binNum--;

                // Do nothing if the camera callbacks are uninitialized
                if (camera_Logical_Images.at(binNum).models.size() == 0)
                {
                    return;
                }

                // Choose the first model
                osrf_gear::Model model = camera_Logical_Images.at(binNum).models.at(0);

                // Check that the part in the list is the part we're looking for
                if (productType == model.type)
                {
                    geometry_msgs::PoseStamped part_pose, goal_pose_bin, goal_pose_agv;
                    part_pose.pose = model.pose;
                    geometry_msgs::Point point = model.pose.position;
                    ROS_WARN("name:= %s x:=%f y:=%f z:=%f", model.type.c_str(), point.x, point.y, point.z);

                    std::string frame = "logical_camera_" + su.unit_id + "_frame";
                    const double centerY = 0.2; // Center of the beam
                    double camY;
                    // Set the y-value for the bin
                    if (su.unit_id == "bin4")
                    {
                        ROS_WARN_STREAM(su.unit_id);
                        camY = 0;
                    }
                    else if (su.unit_id == "bin5")
                    {
                        ROS_WARN_STREAM(su.unit_id);
                        camY = 1;
                    }
                    else if (su.unit_id == "bin6")
                    {
                        ROS_WARN_STREAM(su.unit_id);
                        camY = 2.1;
                    }
                    else
                    {
                        ROS_ERROR("WRONG BIN");
                    }

                    // Move the arm to the bin
                    moveArm(camY);

                    // Then do transform
                    geometry_msgs::TransformStamped binTransform = transformHelper(frame);
                    tf2::doTransform(part_pose, goal_pose_bin, binTransform);

                    // Orient gripper to face down
                    goal_pose_bin.pose.orientation.w = 0.707;
                    goal_pose_bin.pose.orientation.x = 0.0;
                    goal_pose_bin.pose.orientation.y = 0.707;
                    goal_pose_bin.pose.orientation.z = 0.0;

                    // Pick up the part
                    operateGripper(true, goal_pose_bin.pose.position, camY);

                    // Go back to center
                    moveArm(centerY);

                    // Move to tray
                    moveArm(agv_lin);

                    // Add special case for tray 1
                    if (agv_num == 1)
                    {
                        geometry_msgs::Point tray;
                        tray.x = -0.2;
                        tray.y = 0.9;
                        tray.z = 0.1;
                        moveArmAndGripper(agv_lin, tray);
                        operateGripperHelper(false);
                    }
                    else // Orient the arm to drop on the tray
                    {

                        std::string tray_frame = "kit_tray_" + std::to_string(agv_num);
                        geometry_msgs::TransformStamped tray_tf = transformHelper(tray_frame);
                        tf2::doTransform(productDest, goal_pose_agv, tray_tf);

                        goal_pose_agv.pose.orientation.w = 0.707;
                        goal_pose_agv.pose.orientation.x = 0.0;
                        goal_pose_agv.pose.orientation.y = 0.707;
                        goal_pose_agv.pose.orientation.z = 0.0;
                        
                        goal_pose_agv.pose.position.z += z_modify;

                        operateGripper(false, goal_pose_agv.pose.position, agv_lin);
                    }
                    // move home
                    moveArm(centerY);
                }
               
            }
        }
        // Submit the shipment once complete
        osrf_gear::AGVControl submit;
        submit.request.shipment_type = shipment.shipment_type;
        sendOutAGV(submit, agv_num);
        ros::Duration(5).sleep();
        ROS_WARN("Completed Shipment");
    }
    ROS_ERROR("Completed Order");
    ROS_ERROR_STREAM(order_vector.size());
    order_vector.erase(order_vector.begin());
    ROS_ERROR_STREAM(order_vector.size());
}

static trajectory_msgs::JointTrajectory get_trajectory(geometry_msgs::Point dest)
{
    trajectory_msgs::JointTrajectory joint_trajectory = trajectoryHelper();

    double x = dest.x;
    double y = dest.y;
    double z = dest.z;

    T_des[0][3] = x;

    T_des[1][3] = y;

    T_des[2][3] = z;

    T_des[3][3] = 1;

    // The orientation of the end effector so that the end effector is down.
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

    // Fill out the joint trajectory header.
    // Each joint trajectory should have an non-monotonically increasing sequence number.

    for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++)
    {
        for (int indz = 0; indz < joint_states.name.size(); indz++)
        {
            if (joint_trajectory.joint_names[indy] == joint_states.name[indz])
            {
                joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
                break;
            }
        }
    }

    int num_sols = ur_kinematics::inverse((double *)&T_des, (double *)&q_des);

    ROS_INFO_THROTTLE(THROTTLE, "INVS SOLs: %d", num_sols);

    if (num_sols == 0)
    {
        ROS_WARN("INVS SOLs: %d", num_sols);
        joint_trajectory.header.frame_id = "empty";
        return joint_trajectory;
    }

    // Choose the solution that keeps the elbow joint as high as possible
    double target_angle = 3.0 / 2.0 * M_PI; // The elbow is straight up when the shoulder joint is 3/2*pi
    int best_solution_index = -1;
    double best_angle = 10000; // Smaller is better, so this initial score will be beaten by any solution
    for (int i = 0; i < num_sols; i++)
    {
        double pan_angle = q_des[i][0];
        double shoulder_angle = q_des[i][1];
        double wrist_1_angle = q_des[i][3];

        // Ignore solutions where the base or wrist are pointed backwards
        if (abs(M_PI - pan_angle) >= M_PI / 2 || abs(M_PI - wrist_1_angle) >= M_PI / 2)
        {
            continue;
        }

        // Get the angle between the ideal shoulder angle and this solution's shoulder angle
        double dist = std::min(fabs(shoulder_angle - target_angle), 2.0 * M_PI - fabs(shoulder_angle - target_angle));
        if (dist < best_angle)
        {
            best_angle = dist;
            best_solution_index = i;
        }
    }

    for (int indy = 0; indy < 6; indy++)
    {
        joint_trajectory.points[1].positions[indy + 1] = q_des[best_solution_index][indy];
    }

    joint_trajectory.points[1].time_from_start = ros::Duration(5.0);

    return joint_trajectory;
}

static trajectory_msgs::JointTrajectory trajectoryHelper()
{
    trajectory_msgs::JointTrajectory joint_trajectory;
    static int count = 0;
    joint_trajectory.header.seq = count++;
    joint_trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0); // When was this message created.
    joint_trajectory.header.frame_id = "/world";                           // Frame in which this is specified

    // Set the names of the joints being used. All must be present.
    joint_trajectory.joint_names.clear();

    joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
    joint_trajectory.joint_names.push_back("shoulder_pan_joint");
    joint_trajectory.joint_names.push_back("shoulder_lift_joint");
    joint_trajectory.joint_names.push_back("elbow_joint");
    joint_trajectory.joint_names.push_back("wrist_1_joint");
    joint_trajectory.joint_names.push_back("wrist_2_joint");
    joint_trajectory.joint_names.push_back("wrist_3_joint");

    // Set a start and end point.
    joint_trajectory.points.resize(2);

    //  Set the start point to the current position of the joints from joint_states.
    joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());

    // When to start (immediately upon receipt).
    joint_trajectory.points[0].time_from_start = ros::Duration(0.01);

    joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
    joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
    return joint_trajectory;
}

static void moveArmAndGripper(double pose, geometry_msgs::Point dest)
{
    moveArm(pose);
    trajectory_msgs::JointTrajectory joint_trajectory = get_trajectory(dest);
    joint_trajectory.points[1].positions[0] = pose;
    joint_trajectory.points[1].time_from_start = ros::Duration(5.0);
    action_method(joint_trajectory, pose * 3);
}

static void moveArm(double pose)
{
    geometry_msgs::Point homePoint;
    homePoint.x = -0.2;
    homePoint.y = 0.4;
    homePoint.z = 0.2;
    trajectory_msgs::JointTrajectory joint_trajectory = get_trajectory(homePoint);
    for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++)
    {
        for (int indz = 0; indz < joint_states.name.size(); indz++)
        {
            if (joint_trajectory.joint_names[indy] == joint_states.name[indz])
            {
                joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
                joint_trajectory.points[1].positions[indy] = joint_states.position[indz];
                break;
            }
        }
    }
    joint_trajectory.points[1].positions[0] = pose;
    joint_trajectory.points[1].time_from_start = ros::Duration(5.0);
    action_method(joint_trajectory, pose * 3);
}

static void action_method(trajectory_msgs::JointTrajectory joint_trajectory, double duration)
{

    // Make sure there are solutions
    if (joint_trajectory.header.frame_id == "empty")
    {
        ROS_INFO("NO SOLUTIONS");
        return;
    }
    // Create the structure to populate for running the Action Server.
    control_msgs::FollowJointTrajectoryAction joint_trajectory_as;
    // It is possible to reuse the JointTrajectory from above
    joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;

    // added header
    static int count = 0;
    joint_trajectory_as.action_goal.header.seq = count++;
    joint_trajectory_as.action_goal.header.stamp = ros::Time::now();
    joint_trajectory_as.action_goal.header.frame_id = "/world";

    joint_trajectory_as.action_goal.goal_id.stamp = ros::Time::now();
    joint_trajectory_as.action_goal.goal_id.id = std::to_string(count);

    // The header and goal (not the tolerances) of the action must be filled out as well.
    // (rosmsg show control_msgs/FollowJointTrajectoryAction)
    actionlib::SimpleClientGoalState state =
        trajectory_as->sendGoalAndWait(joint_trajectory_as.action_goal.goal,
                                       ros::Duration(30.0), ros::Duration(30.0));
    ROS_INFO("Action Server returned with status: [%i] %s", state.state_,
             state.toString().c_str());
    ros::Duration(std::abs(duration) + 1).sleep();
}

void operateGripperHelper(bool attach)
{
    ros::Duration s(3);
    osrf_gear::VacuumGripperControl srv;
    s.sleep();
    srv.request.enable = attach;
    gripper_client.call(srv);
    s.sleep();
}

static void operateGripper(bool attach, geometry_msgs::Point dest, double pose)
{
    // Move the arm to bin/tray
    moveArmAndGripper(pose, dest);
    // Then grab or drop the part
    operateGripperHelper(attach);
    // Move away from the part depending on if dropping or picking
    if (attach)
    {
        dest.x = dest.z + 0.1;
        dest.z = dest.z + 0.2;
        dest.y = 0;
        action_method(get_trajectory(dest), 2);
    }
    else
    {
        dest.x = 0.2;
        dest.z = dest.z + -0.1;
        dest.y = 0.1;
        action_method(get_trajectory(dest), 2);
    }
}

void gripperStateCallback(const osrf_gear::VacuumGripperStateConstPtr &msg)
{
    gripper_state = *msg;
}

int main(int argc, char *argv[])
{

    joint_states.header.frame_id = "uninitialized";
    ros::init(argc, argv, "lab5");

    ros::NodeHandle n;

    ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
    spinner.start();

    tf2_ros::TransformListener tfListener(tfBuffer);
    trajectory_as =
        new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/ariac/arm1/arm/follow_joint_trajectory/", true);

    // Instantiate the Action Server client
    agv1_client = n.serviceClient<osrf_gear::AGVControl>("/ariac/agv1");
    agv2_client = n.serviceClient<osrf_gear::AGVControl>("/ariac/agv2");

    gripper_client = n.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/arm1/gripper/control");
    ros::Subscriber gripper_state_sub = n.subscribe("/ariac/arm1/gripper/state", 1000, gripperStateCallback);

    gml = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");

    ros::ServiceClient start_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");

    // If it's not already ready, wait for it to be ready.
    // Calling the Service using the client before the server is ready would fail.
    if (!start_client.exists())
    {
        ROS_INFO("Waiting for the competition to be ready...");
        start_client.waitForExistence();
        ROS_INFO("Competition is now ready.");
    }
    ROS_INFO("Requesting competition start...");
    std_srvs::Trigger srv;  // Combination of the "request" and the "response".
    start_client.call(srv); // Call the start Service.
    if (!srv.response.success)
    { // If not successful, print out why.
        ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
    }
    else
    {
        ROS_INFO("Competition started!");
    }

    trajectoryPub = n.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm1/arm/command", 1000);

    ros::Subscriber sub = n.subscribe<osrf_gear::Order>("/ariac/orders", 1000, orderCallback);
    std::vector<ros::Subscriber> binCameras = std::vector<ros::Subscriber>(6);
    std::vector<ros::Subscriber> agvCameras = std::vector<ros::Subscriber>(2);
    std::vector<ros::Subscriber> qualityCameras = std::vector<ros::Subscriber>(2);

    for (int i = 0; i < binCameras.size(); i++)
    {
        char stringCam[100];
        std::sprintf(stringCam, "/ariac/logical_camera_bin%d", i + 1);
        binCameras[i] = n.subscribe<osrf_gear::LogicalCameraImage>(stringCam, 1000,
                       [i](const boost::shared_ptr<const osrf_gear::LogicalCameraImage_<std::allocator<void>>> img)
        {
            camera_Logical_Images.at(i) = *img;
        });
    }

    for (int i = binCameras.size(); i < agvCameras.size() + binCameras.size(); i++)
    {
        char stringCam[100];
        int num = i - binCameras.size();
        std::sprintf(stringCam, "/ariac/logical_camera_agv%d", num + 1);

        agvCameras[num] = n.subscribe<osrf_gear::LogicalCameraImage>(stringCam, 1000,
                         [i](const boost::shared_ptr<const osrf_gear::LogicalCameraImage_<std::allocator<void>>> img)
        {
            camera_Logical_Images.at(i) = *img;
        });
    }

    for (int i = agvCameras.size() + binCameras.size(); i < agvCameras.size() + binCameras.size() + qualityCameras.size(); i++)
    {
        char stringCam[100];
        int num = i - (agvCameras.size() + binCameras.size());
        std::sprintf(stringCam, "/ariac/quality_control_sensor_%d", num + 1);

        qualityCameras[num] = n.subscribe<osrf_gear::LogicalCameraImage>(stringCam, 1000,
                             [i](const boost::shared_ptr<const osrf_gear::LogicalCameraImage_<std::allocator<void>>> img)
        {
            camera_Logical_Images.at(i) = *img;
        });
    }

    ros::Subscriber jointSub = n.subscribe<sensor_msgs::JointState>("/ariac/arm1/joint_states", 1000, jointCallback);

    ROS_INFO("Spinning");
    ros::Rate r(10);
    while (ros::ok)
    {
        printOrderModelPose();
        r.sleep();
    }
}