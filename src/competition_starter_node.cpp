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
std::vector<std::vector<osrf_gear::Model>> camera_data = std::vector<std::vector<osrf_gear::Model>>(10);
//int_vector.clear();
ros::ServiceClient gml ;


void orderCallback(const osrf_gear::Order msg){
    order_vector.push_back(msg);
}

void printOrderModelPose(){
    std::vector<osrf_gear::Shipment> shipments = order_vector.front().shipments;
  
        for(osrf_gear::Shipment shipment : shipments){
       
            for(osrf_gear::Product product : shipment.products){
          
                std::string productType = product.type;
        
                osrf_gear::GetMaterialLocations gmlService;
                gmlService.request.material_type = productType;
                gml.call(gmlService);
             
                for(osrf_gear::StorageUnit su : gmlService.response.storage_units) {
                  
                    const char *binName = su.unit_id.c_str();
                    int binNum;
                    sscanf(binName,"bin%d",&binNum);
                    binNum--;
                   
                    for(osrf_gear::Model model : camera_data[binNum]){
                        if(strstr(productType.c_str(),model.type.c_str())){
                            geometry_msgs::Point point = model.pose.position;
                            ROS_WARN("name:= %s x:=%f y:=%f z:=%f",model.type.c_str(),point.x,point.y,point.z);
                        }
                       
                    }
                }
            }
        }
        //order_vector.erase(order_vector.begin());
}


int main(int argc, char* argv[]){

    
    ros::init(argc, argv, "lab5"); 
    ros::NodeHandle n;
    //ros::NameSpac

    gml= n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");

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

    ros::Subscriber sub = n.subscribe<osrf_gear::Order>("/ariac/orders", 1000,orderCallback);
    std::vector<ros::Subscriber> binCameras = std::vector<ros::Subscriber>(6);
    std::vector<ros::Subscriber> agvCameras= std::vector<ros::Subscriber>(2);
    std::vector<ros::Subscriber> qualityCameras= std::vector<ros::Subscriber>(2);
    
    for(int i=0;i<binCameras.size();i++){
        char stringCam[100];
        std::sprintf(stringCam,"/ariac/logical_camera_bin%d",i+1);
        binCameras[i] = n.subscribe<osrf_gear::LogicalCameraImage>(stringCam,1000, [i](const boost::shared_ptr<const osrf_gear::LogicalCameraImage_<std::allocator<void> > > img)
        {
            for(osrf_gear::Model m : img->models){
                camera_data[i].push_back(m);          
            }
            printOrderModelPose();
            for(osrf_gear::Model m : img->models){
                camera_data[i].pop_back();
            }
        });
    }

    
    for(int i=binCameras.size();i<agvCameras.size()+binCameras.size();i++){
        char stringCam[100];
        std::sprintf(stringCam,"/ariac/logical_camera_agv%d",i+1);
      
        agvCameras[i-binCameras.size()] = n.subscribe<osrf_gear::LogicalCameraImage>(stringCam,1000, [i](const boost::shared_ptr<const osrf_gear::LogicalCameraImage_<std::allocator<void> > > img)
        {
            for(osrf_gear::Model m : img->models){
                camera_data[i].push_back(m);          
            }
        });
    }

    for(int i=agvCameras.size()+binCameras.size();i<agvCameras.size()+binCameras.size()+qualityCameras.size();i++){
        char stringCam[100];
        std::sprintf(stringCam,"/ariac/quality_control_sensor_%d",i+1);
      
        qualityCameras[i-(agvCameras.size()+binCameras.size())] = n.subscribe<osrf_gear::LogicalCameraImage>(stringCam,1000, [i](const boost::shared_ptr<const osrf_gear::LogicalCameraImage_<std::allocator<void> > > img)
        {
            for(osrf_gear::Model m : img->models){
                camera_data[i].push_back(m);          
            }
        });
    }


    
   
    ros::spin();
    
    
    

}