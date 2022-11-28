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
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

std::vector<osrf_gear::Order> order_vector;
sensor_msgs::JointState joint_states;
ros::ServiceClient materialLocations;
std::vector<std::vector<osrf_gear::Model>> camera_data = std::vector<std::vector<osrf_gear::Model>>(10);
ros::ServiceClient gml;
tf2_ros::Buffer tfBuffer;

actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *trajectory_as;

ros::Publisher trajectoryPub;

// std::vector<std::vector<double>>T_pose[4][4], T_des[4][4] = std::vector<std::vector<double>>(4)(4);
// Instantiate variables for use with the kinematic system.
double T_pose[4][4], T_des[4][4];
double q_pose[6], q_des[8][6];

// std::vector<std::vector<double>> q_des[8][6] = std::vector<std::vector<double>>(8)(6);
trajectory_msgs::JointTrajectory desired;

static void get_trajectory_method(trajectory_msgs::JointTrajectory *joint_trajectory, std::string binName, geometry_msgs::Pose model_pose);
static void action_method(trajectory_msgs::JointTrajectory *joint_trajectory);

void orderCallback(const osrf_gear::Order msg)
{
    ROS_INFO("Starting Order callback");
    fflush(stdout);
    order_vector.push_back(msg);
}

void jointCallback(const sensor_msgs::JointState msg)
{
    ROS_INFO("Starting Joint callback");
    fflush(stdout);

    joint_states = msg;
    ROS_INFO("Here 100");
    fflush(stdout);


    std::string str;
    // std::string str2 = "hi";
    ROS_INFO("Here 100");
    fflush(stdout);
    for (std::string s : joint_states.name)
    {
        str = str + " " + s + " ";
        ROS_INFO("Here 101");
        fflush(stdout);
    }
    ROS_INFO_STREAM_THROTTLE(10, str);
    ROS_INFO("Here 102");
    fflush(stdout);
}

void printOrderModelPose()
{
    std::vector<osrf_gear::Shipment> shipments = order_vector.front().shipments;
    ROS_INFO("HERE -10000");
        fflush(stdout);
    for (osrf_gear::Shipment shipment : shipments)
    {
        ROS_INFO("HERE -1000");
        fflush(stdout);

        for (osrf_gear::Product product : shipment.products)
        {

            ROS_INFO("HERE -900");
            fflush(stdout);

            std::string productType = product.type;

            osrf_gear::GetMaterialLocations gmlService;
            gmlService.request.material_type = productType;
            gml.call(gmlService);


            ROS_INFO("HERE -800");
            fflush(stdout);

            for (osrf_gear::StorageUnit su : gmlService.response.storage_units)
            {

                ROS_INFO("HERE -2");
                fflush(stdout);
                const char *binName = su.unit_id.c_str();
                int binNum;
                sscanf(binName, "bin%d", &binNum);
                binNum--;
                ROS_INFO("HERE -3");
                fflush(stdout);



                for (osrf_gear::Model model : camera_data.at(binNum))
                {
                    ROS_INFO("HERE -5");
                    fflush(stdout);
                    ROS_INFO("%s", productType.c_str());
                    fflush(stdout);
                    ROS_INFO("HERE -6");
                    fflush(stdout);
                    ROS_INFO("%s", model.type.c_str());
                    fflush(stdout);
                    ROS_INFO("HERE -7)");
                    fflush(stdout);                    
                    
                    if (strstr(productType.c_str(), model.type.c_str()))
                    {
                        ROS_INFO("HERE- 1");
                        fflush(stdout);
                        geometry_msgs::Point point = model.pose.position;
                        ROS_WARN("name:= %s x:=%f y:=%f z:=%f", model.type.c_str(), point.x, point.y, point.z);
                        fflush(stdout);

                        trajectory_msgs::JointTrajectory joint_trajectory;
                        get_trajectory_method(&joint_trajectory, su.unit_id, model.pose);

                        ROS_INFO("PAIN");
                        fflush(stdout);
                        if (joint_trajectory.header.frame_id == "empty")
                        {
                            ROS_INFO("CONTINUED");
                            fflush(stdout);
                            continue;
                        }
                        ROS_INFO("PAIN");
                        fflush(stdout);

                        ros::Duration r(3);
                        trajectoryPub.publish(joint_trajectory);
                        r.sleep();

                        // action_method(&joint_trajectory);
                        // action_method(&home_trajectory);
                    }
                }
            }
        }
    }
    ROS_INFO("Finished print");
    fflush(stdout);
}

static void get_trajectory_method(trajectory_msgs::JointTrajectory *joint_trajectory, std::string binName, geometry_msgs::Pose model_pose)
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
    fflush(stdout);

    geometry_msgs::PoseStamped part_pose, goal_pose;
    part_pose.pose = model_pose;

    tf2::doTransform(part_pose, goal_pose, tfStamped);

    ROS_INFO("HERE 10");
    fflush(stdout);
    // Add height to the goal pose.
    // goal_pose.pose.position.z += 0.10; // 10 cm above the part
    //  Tell the end effector to rotate 90 degrees around the y-axis (in quaternions...).
    // goal_pose.pose.orientation.w = 0.707;
    // goal_pose.pose.orientation.x = 0.0;
    // goal_pose.pose.orientation.y = 0.707;
    // goal_pose.pose.orientation.z = 0.0;

    // Add height to the goal pose.
    // goal_pose.pose.position.z += 0.10; // 10 cFpointm above the part
    // // Tell the end effector to rotate 90 degrees around the y-axis (in quaternions...).
    // goal_pose.pose.orientation.w = 0.707;
    // goal_pose.pose.orientation.x = 0.0;
    // goal_pose.pose.orientation.y = 0.707;
    // goal_pose.pose.orientation.z = 0.0;

    // double x = goal_pose.pose.position.x;
    //  double y = goal_pose.pose.position.y;
    // double z = goal_pose.pose.position.z;

    // memcpy(q_pose, &joint_states->position[0] + 1, 6);

    geometry_msgs::Point goalPoint = goal_pose.pose.position;

    double x = goalPoint.x;
    double y = goalPoint.y;
    double z = goalPoint.z;

    // x = -0.3;
    //  y = 0.1;
    // z = -0.3;

    // x = x / 2;
    // y = y / 2;
    // z = z / 2;

    ROS_INFO("HERE 11");
    fflush(stdout);

    ROS_INFO("x: %lf, y:%lf, z:%lf", x, y, z);

    ROS_INFO("LAB 6 stuff");
    T_des[0][3] = x;
    ROS_INFO("HERE 0.1");
    fflush(stdout);
    T_des[1][3] = y;
    ROS_INFO("HERE 0.2");
    fflush(stdout);
    T_des[2][3] = z + 0.3;
    ROS_INFO("HERE 0.3");
    fflush(stdout);
    T_des[3][3] = 1;
    // ROS_INFO("HERE 0.4");
    fflush(stdout);

    ROS_INFO("HERE 1");
    fflush(stdout);

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

    ROS_INFO("HERE 12");
    fflush(stdout);

    // //trajectory_msgs::JointTrajectory joint_trajectory;

    // if (joint_states->position[2] == DBL_MIN)
    // {
    //     joint_trajectory->header.frame_id = "empty";
    //     return;
    // }
    ROS_INFO("HERE 12.5");
    fflush(stdout);

    if (joint_states.header.frame_id == "uninitialized") {
        ROS_INFO("HERE 12.75");
        fflush(stdout);
        joint_trajectory->header.frame_id = "empty";
        ROS_INFO("HERE 12.8");
        fflush(stdout);
        return;
    }

    ROS_INFO("HERE 12.7");
    fflush(stdout);
    
    ROS_INFO_STREAM("Joinst States Name: " << joint_states.name[0]);
    fflush(stdout);
    
    q_pose[0] = (joint_states.position)[2];
    q_pose[1] = (joint_states.position)[3];
    q_pose[2] = (joint_states.position)[0];
    q_pose[3] = (joint_states.position)[4];
    q_pose[4] = (joint_states.position)[5];
    q_pose[5] = (joint_states.position)[6];

    ROS_INFO("HERE 13");
    fflush(stdout);

    ur_kinematics::forward((double *)&q_pose, (double *)&T_pose);

    int num_sols = ur_kinematics::inverse((double *)&T_des, (double *)&q_des);

    ROS_INFO("INVS SOLs: %d", num_sols);

    if (num_sols == 0)
    {
        joint_trajectory->header.frame_id = "empty";
        return;
    }
    ROS_INFO("INVS SOLs: %d", num_sols);

    ROS_INFO("HERE 2");
    fflush(stdout);
    ROS_INFO("HERE 13");
    fflush(stdout);

    // Fill out the joint trajectory header.
    // Each joint trajectory should have an non-monotonically increasing sequence number.
    static int count = 0;
    joint_trajectory->header.seq = count++;
    joint_trajectory->header.stamp = ros::Time::now() + ros::Duration(1.0); // When was this message created.
    joint_trajectory->header.frame_id = "/world";      // Frame in which this is specified

    ROS_INFO("HERE 14");
    fflush(stdout);

    // Set the names of the joints being used. All must be present.
    joint_trajectory->joint_names.clear();

    joint_trajectory->joint_names.push_back("linear_arm_actuator_joint");
    joint_trajectory->joint_names.push_back("shoulder_pan_joint");
    joint_trajectory->joint_names.push_back("shoulder_lift_joint");
    joint_trajectory->joint_names.push_back("elbow_joint");
    joint_trajectory->joint_names.push_back("wrist_1_joint");
    joint_trajectory->joint_names.push_back("wrist_2_joint");
    joint_trajectory->joint_names.push_back("wrist_3_joint");

    ROS_INFO("HERE 15");
    fflush(stdout);

    // Set a start and end point.
    joint_trajectory->points.resize(2);

    ROS_INFO("HERE 16");
    fflush(stdout);

    // joint_states = joints;
    //  Set the start point to the current position of the joints from joint_states.
    
    for (int indy = 0; indy < joint_trajectory->joint_names.size(); indy++)
    {
        for (int indz = 0; indz < joint_states.name.size(); indz++)
        {
            if (joint_trajectory->joint_names[indy] == joint_states.name[indz])
            {
                joint_trajectory->points[indy].positions.resize(joint_trajectory->joint_names.size());
                joint_trajectory->points[0].positions[indy] = joint_states.position[indz];
                break;
            }
        }
    }

    ROS_INFO("HERE 17");
    fflush(stdout);

    // When to start (immediately upon receipt).
    joint_trajectory->points[0].time_from_start = ros::Duration(0.0);

    for (int i = 1; i < joint_trajectory->points.size();i++)
    {
        joint_trajectory->points[i].time_from_start = ros::Duration(i);
    }
}

static void action_method(trajectory_msgs::JointTrajectory *joint_trajectory)
{
    // Create the structure to populate for running the Action Server.
    control_msgs::FollowJointTrajectoryAction joint_trajectory_as;
    // It is possible to reuse the JointTrajectory from above
    joint_trajectory_as.action_goal.goal.trajectory = *joint_trajectory;

    // added header
    joint_trajectory_as.action_goal.header = joint_trajectory->header;

    // The header and goal (not the tolerances) of the action must be filled out as well.
    // (rosmsg show control_msgs/FollowJointTrajectoryAction)
    actionlib::SimpleClientGoalState state =
        trajectory_as->sendGoalAndWait(joint_trajectory_as.action_goal.goal,
                                       ros::Duration(30.0), ros::Duration(30.0));
    ROS_INFO("Action Server returned with status: [%i] %s", state.state_,
             state.toString().c_str());
}

int main(int argc, char *argv[])
{

    joint_states.header.frame_id = "uninitialized";
    // trajectory_as();
    ros::init(argc, argv, "lab5");

    ros::NodeHandle n;
    tf2_ros::TransformListener tfListener(tfBuffer);
    trajectory_as =
        new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/ariac/arm1/arm/follow_joint_trajectory/", true);

    // Instantiate the Action Server client

    // a("ariac/arm/follow_joint_trajectory", true);

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
                camera_data.at(i).push_back(m);          
            }
            printOrderModelPose();
            for(osrf_gear::Model m : img->models){
                ROS_INFO("Poping Camera %d", i);
                fflush(stdout);
                camera_data.at(i).pop_back();
            }
            ROS_INFO("End of logicalcam callback");
            fflush(stdout);
            });
            
        ROS_INFO("END OF CALLBACK");
        fflush(stdout);
    }

    for (int i = binCameras.size(); i < agvCameras.size() + binCameras.size(); i++)
    {
        char stringCam[100];
        int num = i - binCameras.size();
        std::sprintf(stringCam, "/ariac/logical_camera_agv%d", num + 1);

        agvCameras[num] = n.subscribe<osrf_gear::LogicalCameraImage>(stringCam, 1000, [i](const boost::shared_ptr<const osrf_gear::LogicalCameraImage_<std::allocator<void>>> img)
                                                                     {
            ROS_INFO("Starting AGV callback");
            fflush(stdout);
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
            ROS_INFO("Starting QC callback");
            fflush(stdout);
            for(osrf_gear::Model m : img->models){
                camera_data[i].push_back(m);          
            } });
    }

    ros::Subscriber jointSub = n.subscribe<sensor_msgs::JointState>("/ariac/arm1/joint_states", 1000, jointCallback);

    ROS_INFO("Spinnging");
    fflush(stdout);
    ros::spin();
}