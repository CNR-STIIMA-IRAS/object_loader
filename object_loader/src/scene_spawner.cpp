#include <ros/ros.h>
#include <object_loader_msgs/addObjects.h>
#include <rosparam_utilities/rosparam_utilities.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "node");
  ros::NodeHandle nh;


  ros::ServiceClient add_obj=nh.serviceClient<object_loader_msgs::addObjects>("add_object_to_scene");
  ROS_INFO_STREAM("Scene spawner is waiting  "<< add_obj.getService());
  add_obj.waitForExistence();
  ROS_INFO("reading object to spawn");
  object_loader_msgs::addObjects srv;


  XmlRpc::XmlRpcValue config;
  if (!nh.getParam("scene_objects",config))
  {
    ROS_ERROR("scene_objects parameter is not defined");
    return 0;
  }


  if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("scene_objects is not a list" );
    return false;
  }

  ROS_DEBUG("there are %zu objects",config.size());



  for(size_t i=0; i < config.size(); i++)
  {
    XmlRpc::XmlRpcValue object = config[i];
    if( object.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_WARN("The element #%zu is not a struct", i);
      continue;
    }
    if( !object.hasMember("type") )
    {
      ROS_WARN("The element #%zu has not the field 'type'", i);
      continue;
    }
    std::string type=rosparam_utilities::toString(object["type"]);

    if( !object.hasMember("frame") )
    {
      ROS_WARN("The element #%zu has not the field 'frame'", i);
      continue;
    }
    std::string frame=rosparam_utilities::toString(object["frame"]);


    ROS_DEBUG("Object type = %s",type.c_str());



    std::vector<double> position;
    if( !rosparam_utilities::getParamVector(object,"position",position) )
    {
      ROS_WARN("pose has not the field 'position'");
      continue;
    }
    assert(position.size()==3);

    std::vector<double> quaternion;
    if( !rosparam_utilities::getParamVector(object,"quaternion",quaternion) )
    {
      ROS_WARN("pose has not the field 'quaternion'");
      continue;
    }
    assert(quaternion.size()==4);


    object_loader_msgs::object obj;

    // if there are multiple object of the same type, add _00


    obj.object_type.data=type;


    obj.pose.pose.position.x=position.at(0);
    obj.pose.pose.position.y=position.at(1);
    obj.pose.pose.position.z=position.at(2);
    obj.pose.pose.orientation.x=quaternion.at(0);
    obj.pose.pose.orientation.y=quaternion.at(1);
    obj.pose.pose.orientation.z=quaternion.at(2);
    obj.pose.pose.orientation.w=quaternion.at(3);
    obj.pose.header.frame_id=frame;

    srv.request.objects.push_back(obj);

  }

  add_obj.call(srv);


  return 0;
}