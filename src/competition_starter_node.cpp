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
#include <map>


std::vector<osrf_gear::Order> order_vector;
 ros::ServiceClient materialLocations;
std::vector<std::vector<geometry_msgs::Pose>> camera_data = std::vector<std::vector<geometry_msgs::Pose>>(10);
//int_vector.clear();


void orderCallback(const osrf_gear::Order msg){
    ROS_INFO("Ordercallback!!!!!");
    order_vector.push_back(msg);
    //ROS_INFO("Order ID: %s",msg.order_id);
   // msg.shipments[0];
   ROS_INFO("HI1");
    std::vector<std::string> types;
    std::vector<osrf_gear::Shipment> shipments = order_vector.front().shipments;
    ROS_INFO("HI2");
    for(osrf_gear::Shipment shipment : shipments){
        ROS_INFO("HI3");
        for(osrf_gear::Product product : shipment.products){
            ROS_INFO("HI4");
            std::string productType = product.type;
            
            osrf_gear::GetMaterialLocations gml;
            gml.request.material_type = productType;
            materialLocations.call(gml);
            ROS_INFO("HI5");
            for(osrf_gear::StorageUnit su : gml.response.storage_units) {
                ROS_WARN("%s\n",su.unit_id);
                ROS_INFO("HI6");
            }
        }
    }

    order_vector.erase(order_vector.begin());
}

void cam1Callback(const osrf_gear::LogicalCameraImage img){
    geometry_msgs::Point point = img.pose.position;
    // ROS_INFO("%f %f %f",point.x,point.y,point.z);
    camera_data[0].push_back(img.pose);
}

void cam2Callback(const osrf_gear::LogicalCameraImage img){
    geometry_msgs::Point point = img.pose.position;
    // ROS_INFO("%f %f %f",point.x,point.y,point.z);
    camera_data[1].push_back(img.pose);
}
void cam3Callback(const osrf_gear::LogicalCameraImage img){
    geometry_msgs::Point point = img.pose.position;
    // ROS_INFO("%f %f %f",point.x,point.y,point.z);
    camera_data[2].push_back(img.pose);
}

void cam4Callback(const osrf_gear::LogicalCameraImage img){
    geometry_msgs::Point point = img.pose.position;
    // ROS_INFO("%f %f %f",point.x,point.y,point.z);
    camera_data[3].push_back(img.pose);
}
void cam5Callback(const osrf_gear::LogicalCameraImage img){
    geometry_msgs::Point point = img.pose.position;
    // ROS_INFO("%f %f %f",point.x,point.y,point.z);
    camera_data[4].push_back(img.pose);
}

void cam6Callback(const osrf_gear::LogicalCameraImage img){
    geometry_msgs::Point point = img.pose.position;
    // ROS_INFO("%f %f %f",point.x,point.y,point.z);
    camera_data[5].push_back(img.pose);
}
void cam1AgvCallback(const osrf_gear::LogicalCameraImage img){
    geometry_msgs::Point point = img.pose.position;
    //ROS_INFO("%f %f %f",point.x,point.y,point.z);
    camera_data[6].push_back(img.pose);
}

void cam2AgvCallback(const osrf_gear::LogicalCameraImage img){
    geometry_msgs::Point point = img.pose.position;
    //ROS_INFO("%f %f %f",point.x,point.y,point.z);
    camera_data[7].push_back(img.pose);
}
void cam1QCCallback(const osrf_gear::LogicalCameraImage img){
    geometry_msgs::Point point = img.pose.position;
    //ROS_INFO("%f %f %f",point.x,point.y,point.z);
    camera_data[8].push_back(img.pose);
}

void cam2QCCallback(const osrf_gear::LogicalCameraImage img){
    geometry_msgs::Point point = img.pose.position;
    //ROS_INFO("%f %f %f",point.x,point.y,point.z);
    camera_data[9].push_back(img.pose);
}
// (void *)(osrf_gear::LogicalCameraImage)
void (* callbacks[])(const osrf_gear::LogicalCameraImage) = {cam1Callback, cam2Callback, cam3Callback, cam4Callback, cam5Callback,
                                cam6Callback, cam1AgvCallback, cam2AgvCallback, cam1QCCallback, cam2QCCallback};


int main(int argc, char* argv[]){

    
    ros::init(argc, argv, "lab5"); 
    ros::NodeHandle n;
    //ros::NameSpac

    ros::ServiceClient start_client =
    n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    // If it's not already ready, wait for it to be ready.
    // Calling the Service using the client before the server is ready would fail.
    if (!start_client.exists()) {
        ROS_INFO("Waiting for the competition to be ready...");
        start_client.waitForExistence();
        ROS_INFO("Competition is now ready.");
    }
    ROS_INFO("Requesting competition start...");
    std_srvs::Trigger srv;  // Combination of the "request" and the "response".
    start_client.call(srv);  // Call the start Service.
    if (!srv.response.success) {  // If not successful, print out why.
        ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
    } else {
        ROS_INFO("Competition started!");
    }

    /*std_srvs::Trigger begin_comp;
    ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    ros::Publisher pubState = n.advertise<std_msgs::String>("/ariac/competition_state",1000);
    
    std_msgs::String stringMsg;
    stringMsg.data = "go";

    //std_srvs::SetBool my_bool_var;
    //my_bool_var.request.data = true;

    bool service_call_succeeded;

    ROS_INFO("hi");

    while(true){

        pubState.publish(stringMsg);

        service_call_succeeded = begin_client.call(begin_comp);
        if(!service_call_succeeded){
            ROS_ERROR("Competition service call failed! Goodness Gracious!!");
        }
        if(strstr(begin_comp.response.message.c_str(), "cannot start if not in 'init' state")){
            ROS_WARN("Competition service returned failure: %s", begin_comp.response.message.c_str());
        }
        else{
            ROS_INFO("Competition service called successfully: %s", begin_comp.response.message.c_str());
            break;
        }
    }*/

     ROS_INFO("hi");
     fflush(stdout);
    ros::Subscriber sub = n.subscribe<osrf_gear::Order>("/ariac/orders", 1000,orderCallback);
    std::vector<ros::Subscriber> binCameras = std::vector<ros::Subscriber>(6);
    std::vector<ros::Subscriber> agvCameras= std::vector<ros::Subscriber>(2);
    std::vector<ros::Subscriber> qualityCameras= std::vector<ros::Subscriber>(2);
    
    ROS_INFO("afterCams");
    fflush(stdout);
    for(int i=0;i<binCameras.size();i++){
        char stringCam[100];
        std::sprintf(stringCam,"/ariac/logROS_INFOical_camera_bin%d",i+1);
        fflush(stdout);
        binCameras[i] = n.subscribe<osrf_gear::LogicalCameraImage>(stringCam,1000, callbacks[i]);
    }

    
    for(int i=0;i<agvCameras.size();i++){
        char stringCam[100];
        std::sprintf(stringCam,"/ariac/logical_camera_agv%d",i+1);
      
        agvCameras[i] = n.subscribe<osrf_gear::LogicalCameraImage>(stringCam,1000, callbacks[i + binCameras.size()]);
    }

    for(int i=0;i<qualityCameras.size();i++){
        char stringCam[100];
        std::sprintf(stringCam,"/ariac/quality_control_sensor_%d",i+1);
      
        qualityCameras[i] = n.subscribe<osrf_gear::LogicalCameraImage>(stringCam,1000, callbacks[i + binCameras.size() + agvCameras.size()]);
    }


    
   
    ros::spin();
    
    
    

}