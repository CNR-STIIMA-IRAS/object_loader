#include <thread>
#include <ros/ros.h>
// MoveIt!
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <geometry_msgs/Pose.h>
#include <shape_msgs/Mesh.h>
#include "geometric_shapes/shape_operations.h"
#include <std_srvs/Trigger.h>

#include <tf/tf.h>
#include <object_loader_msgs/AddObjects.h>
#include <object_loader_msgs/RemoveObjects.h>
#include <object_loader_msgs/AttachObject.h>
#include <object_loader_msgs/DetachObject.h>
#include <rosparam_utilities/rosparam_utilities.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>


const std::string RESET_SCENE_SRV   = "reset_scene";
const std::string ADD_OBJECT_SRV    = "add_object_to_scene";
const std::string REMOVE_OBJECT_SRV = "remove_object_from_scene";
const std::string ATTACH_OBJECT_SRV = "attach_object_to_link";
const std::string DETACH_OBJECT_SRV = "detach_object_to_link";




class PlanningSceneConfigurator
{
  ros::NodeHandle nh_;
  moveit::planning_interface::PlanningSceneInterface       planning_scene_interface_;

  ros::ServiceServer add_object_srv_;
  ros::ServiceServer remove_object_srv_;
  ros::ServiceServer attach_object_srv_;
  ros::ServiceServer detach_object_srv_;
  ros::ServiceServer reset_srv_;
  tf::TransformBroadcaster broadcaster;

  std::map<std::string, moveit_msgs::CollisionObject > objs_map_;
  std::map<std::string, moveit_msgs::ObjectColor     > colors_map_;
  std::vector<tf::StampedTransform> relative_trasforms_;
  std::map<std::string,Eigen::Affine3d,
           std::less<std::string>,
           Eigen::aligned_allocator<std::pair<const std::string,Eigen::Affine3d>>> tf_object_mesh_;

  bool toCollisionObject( const std::string         &collisionObjID
                        , XmlRpc::XmlRpcValue config
                        , const std::string         &reference_frame
                        , const tf::Pose            &pose
                        , moveit_msgs::CollisionObject& collision_object
                        , moveit_msgs::ObjectColor& color)
  {
    collision_object.id = collisionObjID;

    if( config.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("parameter is not a struct");
      return false;
    }

    geometry_msgs::Pose pose_msg;
    Eigen::Affine3d T_reference_object;
    Eigen::Affine3d T_object_mesh;
    tf::poseTFToMsg ( pose, pose_msg );
    T_object_mesh.setIdentity();
    tf::poseMsgToEigen(pose_msg,T_reference_object);
    collision_object.operation = collision_object.ADD;
    collision_object.header.frame_id = reference_frame;


    if( config.hasMember("color") )
    {
      if( (config["color"]).getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR("color has to be an array of 4 elements (r,g,b,alpha)");
        return false;
      }
      std::vector<double> rgba;
      if( !rosparam_utilities::getParamVector(config,"color",rgba) )
      {
        ROS_ERROR("color has to be an array of 4 elements (r,g,b,alpha)");
        return false;
      }
      if (rgba.size()!=4)
      {
        ROS_ERROR("color has to be an array of 4 elements (r,g,b,alpha)");
        return false;
      }
      color.id = collision_object.id;
      color.color.r = rgba.at(0);
      color.color.g = rgba.at(1);
      color.color.b = rgba.at(2);
      color.color.a = rgba.at(3);

    }
    else
    {
      color.id = collision_object.id;
      color.color.r = 255;
      color.color.g = 255;
      color.color.b = 255;
      color.color.a = 1;
    }

    if( config.hasMember("offset_quaternion") )
    {
      if( (config["offset_quaternion"]).getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR("offset has to be an array of 4 elements");
        return false;
      }
      std::vector<double> offset;
      if( !rosparam_utilities::getParamVector(config,"offset_quaternion",offset) )
      {
         ROS_ERROR("offset has to be an array of 4 elements");
        return false;
      }
      if (offset.size()!=4)
      {
         ROS_ERROR("offset has to be an array of 4 elements");
        return false;
      }
      Eigen::Quaterniond q(offset.at(3),offset.at(0),offset.at(1),offset.at(2));
      q.normalize();
      T_object_mesh=q;
    }

    if( config.hasMember("offset") )
    {
      if( (config["offset"]).getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR("offset has to be an array of 3 elements");
        return false;
      }
      std::vector<double> offset;
      if( !rosparam_utilities::getParamVector(config,"offset",offset) )
      {
         ROS_ERROR("offset has to be an array of 3 elements");
        return false;
      }
      if (offset.size()!=3)
      {
         ROS_ERROR("offset has to be an array of 3 elements");
        return false;
      }
      T_object_mesh.translation()(0)=offset.at(0);
      T_object_mesh.translation()(1)=offset.at(1);
      T_object_mesh.translation()(2)=offset.at(2);
    }




    Eigen::Affine3d T_reference_mesh=T_reference_object*T_object_mesh;
    tf::poseEigenToMsg(T_reference_mesh,pose_msg);
    std::vector<tf::StampedTransform> grasp_trasforms;
    if (config.hasMember("grasp_poses"))
    {
      XmlRpc::XmlRpcValue grasps=config["grasp_poses"];
      for (int ig=0;ig<grasps.size();ig++)
      {
        XmlRpc::XmlRpcValue grasp=grasps[ig];
        std::string tool=grasp["tool"];
        std::string grasp_name=collisionObjID+"_grasp"+std::to_string(ig)+"_"+tool;

        if( (grasp["quaternion"]).getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
          ROS_ERROR("offset has to be an array of 4 elements");
          return false;
        }
        std::vector<double> quat;
        if( !rosparam_utilities::getParamVector(grasp,"quaternion",quat) )
        {
           ROS_ERROR("offset has to be an array of 4 elements");
          return false;
        }
        if (quat.size()!=4)
        {
           ROS_ERROR("offset has to be an array of 4 elements");
          return false;
        }
        tf::StampedTransform tf;
        tf::Quaternion q(quat.at(0),quat.at(1),quat.at(2),quat.at(3));
        tf.setRotation(q);

        if( (grasp["position"]).getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
          ROS_ERROR("offset has to be an array of 3 elements");
          return false;
        }
        std::vector<double> position;
        if( !rosparam_utilities::getParamVector(grasp,"position",position) )
        {
           ROS_ERROR("offset has to be an array of 3 elements");
          return false;
        }
        if (position.size()!=3)
        {
           ROS_ERROR("offset has to be an array of 3 elements");
          return false;
        }
        tf::Vector3 orig(position.at(0),position.at(1),position.at(2));
        tf.setOrigin(orig);
        tf.frame_id_=collisionObjID;
        tf.child_frame_id_=grasp_name;
        grasp_trasforms.push_back(tf);
      }
    }
    if( config.hasMember("mesh") )
    {
      XmlRpc::XmlRpcValue mesh_config=config["mesh"];
      if( mesh_config.getType() != XmlRpc::XmlRpcValue::TypeString)
      {
        ROS_ERROR("mesh has to be a string");
        return false;
      }

      std::string path=rosparam_utilities::toString(mesh_config);


      Eigen::Vector3d scale = Eigen::Vector3d(1,1,1);
      if (config.hasMember("scale"))
      {
        if( (config["scale"]).getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
          ROS_ERROR("scale has to be an array of 3 elements");
          return false;
        }
        std::vector<double> scale_v;
        if( !rosparam_utilities::getParamVector(config,"scale",scale_v) )
        {
          ROS_ERROR("scale has to be an array of 3 elements");
          return false;
        }
        if (scale_v.size()!=3)
        {
          ROS_ERROR("scale has to be an array of 3 elements");
          return false;
        }
        scale(0)=scale_v.at(0);
        scale(1)=scale_v.at(1);
        scale(2)=scale_v.at(2);
      }
      shapes::Mesh* m = shapes::createMeshFromResource ( path, scale );

      shape_msgs::Mesh mesh;
      shapes::ShapeMsg mesh_msg;
      shapes::constructMsgFromShape ( m, mesh_msg );
      mesh = boost::get<shape_msgs::Mesh> ( mesh_msg );

      collision_object.meshes.resize ( 1 );
      collision_object.mesh_poses.resize ( 1 );
      collision_object.meshes[0] = mesh;
      collision_object.mesh_poses[0] = pose_msg;
      collision_object.header.frame_id = reference_frame;
      tf_object_mesh_.insert(std::pair<std::string,Eigen::Affine3d>(collisionObjID,T_object_mesh));
      for (auto tf: grasp_trasforms)
        relative_trasforms_.push_back(tf);
      return true;
    }
    if( config.hasMember("box") )
    {
      shape_msgs::SolidPrimitive primitive;
      std::vector<double> size;
      if( !rosparam_utilities::getParamVector(config,"box",size) )
      {
        ROS_ERROR("box has to be an array of 3 elements");
        return false;
      }
      if (size.size()!=3)
      {
        ROS_ERROR("box has to be an array of 3 elements");
        return false;
      }

      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[0] = size.at(0);
      primitive.dimensions[1] = size.at(1);
      primitive.dimensions[2] = size.at(2);

      collision_object.primitive_poses.push_back(pose_msg);
      collision_object.primitives.push_back(primitive);
      tf_object_mesh_.insert(std::pair<std::string,Eigen::Affine3d>(collisionObjID,T_object_mesh));
      for (auto tf: grasp_trasforms)
        relative_trasforms_.push_back(tf);
      return true;
    }
    ROS_ERROR_STREAM("configuration not recognized\n"<< config);
    return false;

  }
  
  bool addObjects( object_loader_msgs::AddObjects::Request&  req
                   , object_loader_msgs::AddObjects::Response& res )
  {
    std::vector< moveit_msgs::CollisionObject > objs;
    std::vector< moveit_msgs::ObjectColor     > colors;
    std::vector<std::string > known_objects = planning_scene_interface_.getKnownObjectNames();
    std::map<std::string,int> types;
    for (const std::string& object_id: known_objects)
    {
      std::string type;
      std::size_t found = object_id.find_last_of("_");
      if (found==std::string::npos)
        type=object_id;
      else
        type=object_id.substr(0,found);

      std::map<std::string,int>::iterator it =types.find(type);
      if (it==types.end())
        types.insert(std::pair<std::string,int>(type,0));


      it =types.find(type);
      int ia = object_id.back() - '0';
      if (ia>=0 && ia<=9)
        it->second++;


    }
    for (auto obj : req.objects)
    {
      std::string type = obj.object_type;
      
      
      std::string id;
      std::map<std::string,int>::iterator it =types.find(type);
      if (it==types.end())
      {
        types.insert(std::pair<std::string,int>(type,1));
        id=type+"_0";
      }
      else
      {
        id=type+"_"+std::to_string(it->second++);
      }
      res.ids.push_back(id);

      ROS_DEBUG_STREAM("adding "<<id);
      
      tf::Pose T_0_hc ;
      tf::poseMsgToTF( obj.pose.pose, T_0_hc );
      
      XmlRpc::XmlRpcValue config;
      if(!nh_.getParam("/manipulation_objects/"+obj.object_type,config))
      {
        ROS_ERROR_STREAM("param "<<nh_.getNamespace()<<"/"<< obj.object_type <<" not found");
        res.success = false;
        return true;
      }
      
      moveit_msgs::ObjectColor color;
      moveit_msgs::CollisionObject collision_object;
      if (!toCollisionObject( id, config, obj.pose.header.frame_id, T_0_hc,collision_object,color))
      {
        ROS_ERROR("error loading object %s",obj.object_type.c_str());
        continue;
      }
      objs.push_back( collision_object );
      colors.push_back(color);

      objs_map_.insert(std::pair<std::string, moveit_msgs::CollisionObject >(id,collision_object));
      colors_map_.insert(std::pair<std::string, moveit_msgs::ObjectColor >(id,color));

    }
    

    if (!planning_scene_interface_.applyCollisionObjects(objs,colors))
    {
      ROS_FATAL_STREAM("Failed in uploading the collision objects");
      res.success = false;
    }
    else
    {
      ROS_DEBUG_STREAM("Ok! Objects loaded in the planning scene");
      res.success = true;
    }

    return true;
  }
  
  bool removeObjects( object_loader_msgs::RemoveObjects::Request&  req
                      , object_loader_msgs::RemoveObjects::Response& res )
  {
    std::vector<std::string > v;
    for (auto obj : req.obj_ids)
      v.push_back(obj);
    planning_scene_interface_.removeCollisionObjects ( v );
    res.success = true;
    return true;
  }
  
  bool resetScene( std_srvs::Trigger::Request&   req
                   , std_srvs::Trigger::Response&  res )
  {
    std::vector<std::string > v = planning_scene_interface_.getKnownObjectNames();
    

    
    std::map<std::string, moveit_msgs::AttachedCollisionObject> aco = planning_scene_interface_.getAttachedObjects( );
    for (auto c:aco)
    { 
      object_loader_msgs::DetachObject msg;
      msg.request.obj_id = c.first;
      if(!detachObject(msg.request,msg.response))
        ROS_ERROR_STREAM("Error in detaching "<<c.first);
     }
    
    for (auto c:aco)
    {
      v.push_back(c.first);
    }

    for (const std::string obj: v)
    {
      // remove from map
      ROS_DEBUG_STREAM("erasing " << obj);
      if (objs_map_.find(obj)!=objs_map_.end())
        objs_map_.erase(obj);
    }
    planning_scene_interface_.removeCollisionObjects ( v );

    return (res.success = true);
    
  }


  bool attachObject(object_loader_msgs::AttachObject::Request& req,
                    object_loader_msgs::AttachObject::Response& res)
  {
    std::map<std::string, moveit_msgs::CollisionObject >::iterator obj_it= objs_map_.find(req.obj_id);
    if (obj_it==objs_map_.end())
    {
      ROS_WARN("object id %s is not managed by the object loader",req.obj_id.c_str());
      res.success=false;
      return true;
    }
    std::map<std::string, moveit_msgs::ObjectColor >::iterator color_it= colors_map_.find(req.obj_id);
    if (color_it==colors_map_.end())
    {
      ROS_WARN("object id %s is not managed by the object loader",req.obj_id.c_str());
      res.success=false;
      return true;
    }


    std::vector<std::string> ids;
    ids.push_back(req.obj_id);
    std::map<std::string, moveit_msgs::CollisionObject> collision_objects=planning_scene_interface_.getObjects(ids);
    if (collision_objects.size()==0)
    {
      ROS_WARN("object id %s is not in the scene",req.obj_id.c_str());
      res.success=false;
      return true;
    }

    moveit_msgs::CollisionObject obj=collision_objects.at(req.obj_id);
    obj_it->second=obj;
    obj_it->second.operation=moveit_msgs::CollisionObject::REMOVE;
    planning_scene_interface_.applyCollisionObject(obj_it->second);

    moveit_msgs::AttachedCollisionObject picked_object;
    picked_object.object=obj_it->second;

    picked_object.link_name=req.link_name;
    picked_object.object.operation=moveit_msgs::CollisionObject::ADD;
    planning_scene_interface_.applyAttachedCollisionObject(picked_object);


    res.success=true;
    return true;

  }

  bool detachObject(object_loader_msgs::DetachObject::Request& req,
                    object_loader_msgs::DetachObject::Response& res)
  {
    std::map<std::string, moveit_msgs::CollisionObject >::iterator obj_it= objs_map_.find(req.obj_id);
    if (obj_it==objs_map_.end())
    {
      ROS_WARN("object id %s is not managed by the object loader",req.obj_id.c_str());
      res.success=false;
      return true;
    }
    std::map<std::string, moveit_msgs::ObjectColor >::iterator color_it= colors_map_.find(req.obj_id);
    if (color_it==colors_map_.end())
    {
      ROS_WARN("object id %s is not managed by the object loader",req.obj_id.c_str());
      res.success=false;
      return true;
    }

    std::vector<std::string> ids;
    ids.push_back(req.obj_id);
    std::map<std::string, moveit_msgs::AttachedCollisionObject> attached_objs=planning_scene_interface_.getAttachedObjects(ids);
    if (attached_objs.size()==0)
    {
      ROS_WARN("object id %s is not in the scene",req.obj_id.c_str());
      res.success=false;
      return true;
    }

    moveit_msgs::AttachedCollisionObject placed_object=attached_objs.at(req.obj_id);
    placed_object.object.operation=moveit_msgs::CollisionObject::REMOVE;
    planning_scene_interface_.applyAttachedCollisionObject(placed_object);
    placed_object.object.operation=moveit_msgs::CollisionObject::ADD;
    planning_scene_interface_.applyCollisionObject(placed_object.object,color_it->second.color);
    obj_it->second=placed_object.object;

    res.success=true;
    return true;

  }

public:
  PlanningSceneConfigurator(  )
    : nh_  ()
  {

    add_object_srv_    = nh_.advertiseService(ADD_OBJECT_SRV    , &PlanningSceneConfigurator::addObjects   , this);
    remove_object_srv_ = nh_.advertiseService(REMOVE_OBJECT_SRV , &PlanningSceneConfigurator::removeObjects, this);
    attach_object_srv_ = nh_.advertiseService(ATTACH_OBJECT_SRV , &PlanningSceneConfigurator::attachObject ,  this);
    detach_object_srv_ = nh_.advertiseService(DETACH_OBJECT_SRV , &PlanningSceneConfigurator::detachObject ,  this);
    reset_srv_         = nh_.advertiseService(RESET_SCENE_SRV   , &PlanningSceneConfigurator::resetScene   , this);
  }

  ~PlanningSceneConfigurator()
  {
  }

  void updateTF()
  {

    std::vector<std::string > object_names = planning_scene_interface_.getKnownObjectNames();
    std::map<std::string, moveit_msgs::AttachedCollisionObject> attached_objects = planning_scene_interface_.getAttachedObjects();

    std::vector<std::string> object_ids;
    std::vector<std::string> attached_object_ids;
    for (const std::pair<std::string, moveit_msgs::CollisionObject >& obj:  objs_map_)
    {
      if (std::find(object_names.begin(),object_names.end(),obj.first)<object_names.end())
      {
        object_ids.push_back(obj.first);
      }
      else if (attached_objects.find(obj.first)!=attached_objects.end())
      {
        attached_object_ids.push_back(obj.first);
      }
      else
        ROS_ERROR_STREAM_THROTTLE(1,"object "+obj.first+" in not in the scene");
    }

    std::map<std::string, geometry_msgs::Pose> object_poses=planning_scene_interface_.getObjectPoses(object_ids);
    std::map<std::string, moveit_msgs::CollisionObject> objects = planning_scene_interface_.getObjects(object_ids);

    for (const std::string& object_name: object_ids)
    {
      std::string parent=objects.at(object_name).header.frame_id;
      geometry_msgs::Pose pose=object_poses.at(object_name);

      Eigen::Affine3d T_parent_mesh;
      tf::poseMsgToEigen(pose,T_parent_mesh);
      Eigen::Affine3d T_obj_mesh=tf_object_mesh_.at(object_name);
      Eigen::Affine3d T_parent_obj=T_parent_mesh*T_obj_mesh.inverse();
      tf::StampedTransform tf;
      tf.frame_id_=parent;
      tf.child_frame_id_=object_name;
      tf.stamp_=ros::Time::now();
      tf::poseEigenToTF(T_parent_obj,tf);
      broadcaster.sendTransform(tf);

    }
    for (const std::string& object_name: attached_object_ids)
    {
      geometry_msgs::Pose pose;
      if (attached_objects.at(object_name).object.primitive_poses.size()==1)
        pose=attached_objects.at(object_name).object.primitive_poses.at(0);
      else if (attached_objects.at(object_name).object.mesh_poses.size()==1)
        pose=attached_objects.at(object_name).object.mesh_poses.at(0);
      else
      {
        ROS_INFO_STREAM("TF publishing works only with one and only one primitive_pose or one and only one mesh_pose. Object "+object_name+" does not satisfy this condition");
        continue;
      }
      std::string parent=attached_objects.at(object_name).link_name;

      Eigen::Affine3d T_parent_mesh;
      tf::poseMsgToEigen(pose,T_parent_mesh);
      Eigen::Affine3d T_obj_mesh=tf_object_mesh_.at(object_name);
      Eigen::Affine3d T_parent_obj=T_parent_mesh*T_obj_mesh.inverse();
      tf::StampedTransform tf;
      tf.frame_id_=parent;
      tf.child_frame_id_=object_name;
      tf.stamp_=ros::Time::now();
      tf::poseEigenToTF(T_parent_obj,tf);
      broadcaster.sendTransform(tf);
    }

    for (tf::StampedTransform tf: relative_trasforms_)
    {
      tf.stamp_=ros::Time::now();
      broadcaster.sendTransform(tf);
    }
  }
};

int main(int argc, char** argv)
{

  ros::init(argc,argv,"object_loader");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;
  ros::ServiceClient moveit_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
  ROS_INFO_STREAM("context manager is waiting MoveIt!");
  moveit_client.waitForExistence();
  ROS_INFO("connected with MoveIt!");

  PlanningSceneConfigurator planning_scene_configurator;

  ros::Rate lp(10);
  while (ros::ok())
  {
    lp.sleep();
    planning_scene_configurator.updateTF();
  }
  ros::waitForShutdown();
  return 0;
}
