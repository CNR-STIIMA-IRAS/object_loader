#pragma once

#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

#include <geometry_msgs/Pose.h>
#include <std_srvs/Trigger.h>

#include <tf/tf.h>

#include <object_loader_msgs/addObjects.h>
#include <object_loader_msgs/removeObjects.h>

class SceneObjectsManager
{
protected:
  ros::NodeHandle nh_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  ros::ServiceServer add_object_srv_;
  ros::ServiceServer remove_object_srv_;
  ros::ServiceServer reset_srv_;  
  
  bool applyAndCheckPS( ros::NodeHandle nh, std::vector<moveit_msgs::CollisionObject> cov, std::vector<moveit_msgs::ObjectColor> colors , ros::Duration timeout);

  moveit_msgs::CollisionObject toCollisionObject( const std::string &collisionObjID
                                                , const std::string &path_to_mesh
                                                , const std::string &reference_frame
                                                , const tf::Pose &pose
                                                , const Eigen::Vector3d scale = Eigen::Vector3d(1,1,1));
  
  bool addObjects( object_loader_msgs::addObjects::Request&  req
                 , object_loader_msgs::addObjects::Response& res );
  
  bool removeObjects( object_loader_msgs::removeObjects::Request&  req
                    , object_loader_msgs::removeObjects::Response& res );
  
  bool resetScene( std_srvs::Trigger::Request&   req, std_srvs::Trigger::Response&  res );
  
public:
  SceneObjectsManager ( ros::NodeHandle nh );
};
