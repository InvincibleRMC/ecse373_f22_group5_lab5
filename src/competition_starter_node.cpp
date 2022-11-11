#include "std_srvs/Trigger.h"
#include <ros/ros.h>
#include "std_srvs/SetBool.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/StorageUnit.h"
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

std::vector<osrf_gear::Order> order_vector;
sensor_msgs::JointState joint_states;
ros::ServiceClient materialLocations;
std::vector<std::vector<osrf_gear::Model>> camera_data = std::vector<std::vector<osrf_gear::Model>>(10);
ros::ServiceClient gml;
tf2_ros::Buffer tfBuffer;

ros::Publisher trajectoryPub;

// std::vector<std::vector<double>>T_pose[4][4], T_des[4][4] = std::vector<std::vector<double>>(4)(4);
// Instantiate variables for use with the kinematic system.
double T_pose[4][4], T_des[4][4];
double q_pose[6], q_des[8][6];

// std::vector<std::vector<double>> q_des[8][6] = std::vector<std::vector<double>>(8)(6);
trajectory_msgs::JointTrajectory desired;

static void join_trajectory_method(std::string binName, geometry_msgs::Pose model_pose);
static void action_method(std::string binName, geometry_msgs::Pose model_pose);

void orderCallback(const osrf_gear::Order msg)
{
    order_vector.push_back(msg);
}

void jointCallback(const sensor_msgs::JointState j)
{
    joint_states = j;

    std::string str;
    std::string str2 = "hi";
    for (std::string s : j.name)
    {
        str = str + " " + s + " ";
    }

    memcpy(q_pose, &joint_states.position[0] + 1, 6);
    ur_kinematics::forward((double *)&q_pose, (double *)&T_pose);
    ROS_INFO_STREAM_THROTTLE(10, str);
}

void printOrderModelPose()
{
    std::vector<osrf_gear::Shipment> shipments = order_vector.front().shipments;

    for (osrf_gear::Shipment shipment : shipments)
    {

        for (osrf_gear::Product product : shipment.products)
        {

            std::string productType = product.type;

            osrf_gear::GetMaterialLocations gmlService;
            gmlService.request.material_type = productType;
            gml.call(gmlService);

            for (osrf_gear::StorageUnit su : gmlService.response.storage_units)
            {

                const char *binName = su.unit_id.c_str();
                int binNum;
                sscanf(binName, "bin%d", &binNum);
                binNum--;

                for (osrf_gear::Model model : camera_data[binNum])
                {
                    if (strstr(productType.c_str(), model.type.c_str()))
                    {
                        // ROS_INFO("HERE- 1");
                        // fflush(stdout);
                        geometry_msgs::Point point = model.pose.position;
                        ROS_WARN("name:= %s x:=%f y:=%f z:=%f", model.type.c_str(), point.x, point.y, point.z);

                        join_trajectory_method(su.unit_id, model.pose);
                        // action_method();
                    }
                }
            }
        }
    }
}

static void join_trajectory_method(std::string binName, geometry_msgs::Pose model_pose)
{

    ROS_INFO("HERE 8");
    fflush(stdout);

    std::string frame = "logical_camera_" + binName + "_frame";
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
    ROS_INFO("here 9");

    geometry_msgs::PoseStamped part_pose, goal_pose;
    part_pose.pose = model_pose;

    tf2::doTransform(part_pose, goal_pose, tfStamped);

    // Add height to the goal pose.
    goal_pose.pose.position.z += 0.10; // 10 cm above the part
                                       //  Tell the end effector to rotate 90 degrees around the y-axis (in quaternions...).
    goal_pose.pose.orientation.w = 0.707;
    goal_pose.pose.orientation.x = 0.0;
    goal_pose.pose.orientation.y = 0.707;
    goal_pose.pose.orientation.z = 0.0;

    // Add height to the goal pose.
    // goal_pose.pose.position.z += 0.10; // 10 cm above the part
    // // Tell the end effector to rotate 90 degrees around the y-axis (in quaternions...).
    // goal_pose.pose.orientation.w = 0.707;
    // goal_pose.pose.orientation.x = 0.0;
    // goal_pose.pose.orientation.y = 0.707;
    // goal_pose.pose.orientation.z = 0.0;

    // double x = goal_pose.pose.position.x;
    //  double y = goal_pose.pose.position.y;
    // double z = goal_pose.pose.position.z;

    geometry_msgs::Point point = model_pose.position;

    double x = point.x;
    double y = point.y;
    double z = point.z;

    ROS_INFO("x: %lf, y:%lf, z:%lf", x, y, z);

    ROS_INFO("LAB 6 stuff");
    T_des[0][3] = x;
    // ROS_INFO("HERE 0.1");
    // fflush(stdout);
    T_des[1][3] = y;
    // ROS_INFO("HERE 0.2");
    // fflush(stdout);
    T_des[2][3] = z + 0.3;
    // ROS_INFO("HERE 0.3");
    // fflush(stdout);
    T_des[3][3] = 1;
    // ROS_INFO("HERE 0.4");
    // fflush(stdout);

    // ROS_INFO("HERE 1");
    // fflush(stdout);

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

    // T_des[0][0] = 1.0; T_des[0][1] = 0.0; T_des[0][2] = 0.0;
    // T_des[1][0] = 0.0; T_des[1][1] = 1.0; T_des[1][2] = 0.0;
    // T_des[2][0] = 0.0; T_des[2][1] = 0.0; T_des[2][2] = 1.0;
    // T_des[3][0] = 0.0; T_des[3][1] = 0.0; T_des[3][2] = 0.0;

    int num_sols = ur_kinematics::inverse((double *)&T_des, (double *)&q_des);
    ROS_INFO("INVS SOLs: %d", num_sols);

    if (num_sols == 0)
    {
        return;
    }
    ROS_INFO("INVS SOLs: %d", num_sols);
    trajectory_msgs::JointTrajectory joint_trajectory;

    // ROS_INFO("HERE 2");
    // fflush(stdout);

    // Fill out the joint trajectory header.
    // Each joint trajectory should have an non-monotonically increasing sequence number.
    static int count = 0;
    joint_trajectory.header.seq = count++;
    joint_trajectory.header.stamp = ros::Time::now(); // When was this message created.
    joint_trajectory.header.frame_id = "/world";      // Frame in which this is specified

    // ROS_INFO("HERE 3");
    // fflush(stdout);

    // Set the names of the joints being used. All must be present.
    joint_trajectory.joint_names.clear();
    joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
    joint_trajectory.joint_names.push_back("shoulder_pan_joint");
    joint_trajectory.joint_names.push_back("shoulder_lift_joint");
    joint_trajectory.joint_names.push_back("elbow_joint");
    joint_trajectory.joint_names.push_back("wrist_1_joint");
    joint_trajectory.joint_names.push_back("wrist_2_joint");
    joint_trajectory.joint_names.push_back("wrist_3_joint");

    // ROS_INFO("HERE 4");
    // fflush(stdout);

    // Set a start and end point.
    joint_trajectory.points.resize(2);

    // ROS_INFO("HERE 5");
    // fflush(stdout);

    // joint_states = joints;
    //  Set the start point to the current position of the joints from joint_states.
    joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
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

    // ROS_INFO("HERE 6");
    // fflush(stdout);

    // When to start (immediately upon receipt).
    joint_trajectory.points[0].time_from_start = ros::Duration(0.0);
    // ROS_INFO("HERE 6.1s");
    // fflush(stdout);
    // Must select which of the num_sols solutions to use. Just start with the first.
    int q_des_indx = 0;
    // Set the end point for the movement
    joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
    // Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution.
    //  ROS_INFO("HERE 6.5");
    // fflush(stdout);

    joint_states.position.at(1);
    // ROS_INFO("HERE 6.55");
    // fflush(stdout);

    (joint_trajectory.points.at(1)).positions.at(0) = joint_states.position.at(1);
    // The actuators are commanded in an odd order, enter the joint positions in the correct positions
    // ROS_INFO("HERE 6.6");
    // fflush(stdout);
    for (int indy = 0; indy < 6; indy++)
    {
        joint_trajectory.points[1].positions[indy + 1] = q_des[q_des_indx][indy];
    }
    // ROS_INFO("HERE 6.7");
    // fflush(stdout);

    // ROS_INFO("HERE 7");
    // fflush(stdout);
    // How long to take for the movement.
    joint_trajectory.points[1].time_from_start = ros::Duration(1.0);

    trajectoryPub.publish(joint_trajectory);

    // ROS_INFO("HERE 8");
    // fflush(stdout);

    // std::string frame = "logical_camera_"+su.unit_id+ "_frame";
    // geometry_msgs::TransformStamped tfStamped;
    // try {
    //     tfStamped = tfBuffer.lookupTransform("arm1_vacuum_gripper_link",frame,
    //     ros::Time(0.0), ros::Duration(1.0));
    //     ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),
    //     tfStamped.child_frame_id.c_str());
    // } catch (tf2::TransformException &ex) {
    //     ROS_ERROR("%s", ex.what());
    // }

    // geometry_msgs::PoseStamped part_pose, goal_pose;
    // part_pose.pose = model.pose;

    // tf2::doTransform(part_pose, goal_pose, tfStamped);

    // // Add height to the goal pose.
    // goal_pose.pose.position.z += 0.10; // 10 cm above the part
    // // Tell the end effector to rotate 90 degrees around the y-axis (in quaternions...).
    // goal_pose.pose.orientation.w = 0.707;
    // goal_pose.pose.orientation.x = 0.0;
    // goal_pose.pose.orientation.y = 0.707;
    // goal_pose.pose.orientation.z = 0.0;

    // tf2::doTransform(part_pose, goal_pose, tfStamped);

    // throw std::runtime_error("hi");
}


static void action_method(std::string binName, geometry_msgs::Pose model_pose){}

int main(int argc, char *argv[])
{

    // T_pose.resize(4, std::vector<double>(4));
    // T_des.resize(4, std::vector<double>(4));
    // q_des.resize(8, std::vector<double>(6));

    ROS_INFO("T_desing");
    fflush(stdout);
    // T_des.at(0).at(3) = 0.1;
    // T_des[1][3] = 1.1;
    // T_des[2][3] = 2.1;
    // T_des[3][3] = 3.1;
    // ros::Duration(1.0).sleep();
    ROS_INFO("T_desed");
    fflush(stdout);

    ros::init(argc, argv, "lab5");
    ros::NodeHandle n;
    tf2_ros::TransformListener tfListener(tfBuffer);

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
        binCameras[i] = n.subscribe<osrf_gear::LogicalCameraImage>(stringCam, 1000, [i](const boost::shared_ptr<const osrf_gear::LogicalCameraImage_<std::allocator<void>>> img)
                                                                   {
            for(osrf_gear::Model m : img->models){
                camera_data[i].push_back(m);          
            }
            printOrderModelPose();
            for(osrf_gear::Model m : img->models){
                camera_data[i].pop_back();
            } });
    }

    for (int i = binCameras.size(); i < agvCameras.size() + binCameras.size(); i++)
    {
        char stringCam[100];
        int num = i - binCameras.size();
        std::sprintf(stringCam, "/ariac/logical_camera_agv%d", num + 1);

        agvCameras[num] = n.subscribe<osrf_gear::LogicalCameraImage>(stringCam, 1000, [i](const boost::shared_ptr<const osrf_gear::LogicalCameraImage_<std::allocator<void>>> img)
                                                                     {
            for(osrf_gear::Model m : img->models){
                camera_data[i].push_back(m);          
            } });
    }

    for (int i = agvCameras.size() + binCameras.size(); i < agvCameras.size() + binCameras.size() + qualityCameras.size(); i++)
    {
        char stringCam[100];
        int num = i - (agvCameras.size() + binCameras.size());
        std::sprintf(stringCam, "/ariac/quality_control_sensor_%d", num + 1);

        qualityCameras[num] = n.subscribe<osrf_gear::LogicalCameraImage>(stringCam, 1000, [i](const boost::shared_ptr<const osrf_gear::LogicalCameraImage_<std::allocator<void>>> img)
                                                                         {
            for(osrf_gear::Model m : img->models){
                camera_data[i].push_back(m);          
            } });
    }

    ros::Subscriber jointSub = n.subscribe<sensor_msgs::JointState>("/ariac/arm1/joint_states", 1000, jointCallback);

    ros::spin();
}