#include <moveit_msgs/ApplyPlanningScene.h>

#include <shape_msgs/Mesh.h>
#include <geometric_shapes/shape_operations.h>
#include <object_loader/scene_objects_manager.h>

const std::string RESET_SCENE_SRV   = "reset_scene";
const std::string ADD_OBJECT_SRV    = "add_object_to_scene";
const std::string REMOVE_OBJECT_SRV = "remove_object_from_scene";

bool SceneObjectsManager::applyAndCheckPS( ros::NodeHandle nh
                                        , std::vector<moveit_msgs::CollisionObject> cov
                                        , std::vector<moveit_msgs::ObjectColor> colors , ros::Duration timeout)
{

  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene> ( "planning_scene", 1 );

  while ( planning_scene_diff_publisher.getNumSubscribers() < 1 )
  {
    ros::WallDuration sleep_t ( 0.5 );
    sleep_t.sleep();
  }

  moveit_msgs::PlanningScene planning_scene_msg;

  for(auto const & color : colors )
    planning_scene_msg.object_colors.push_back ( color );


  for ( moveit_msgs::CollisionObject co: cov )
      planning_scene_msg.world.collision_objects.push_back ( co );

  planning_scene_msg.is_diff = true;

  ROS_INFO_STREAM("Update planning scene");
  ros::ServiceClient planning_scene_diff_client = nh.serviceClient<moveit_msgs::ApplyPlanningScene> ( "apply_planning_scene" );
  planning_scene_diff_client.waitForExistence();

  moveit_msgs::ApplyPlanningScene srv;
  srv.request.scene = planning_scene_msg;
  planning_scene_diff_client.call ( srv );

  ROS_INFO_STREAM("Wait for updated planning scene...");
  ros::Time st = ros::Time::now();
  while( ros::ok() )
  {
    auto const obj_names = planning_scene_interface_.getKnownObjectNames( );
    bool loaded = true;
    for( const auto & co : cov )
    {
      auto const it = std::find_if( obj_names.begin(), obj_names.end(), [ & ]( const std::string s ) { return co.id == s;} );
      loaded &= (obj_names.end() != it);

    }
    if( loaded )
    {
      ROS_INFO_STREAM("Updated planning scene.");
      break;
    }
    else
    {
      if( (ros::Time::now() - st) > timeout)
      {
        ROS_FATAL_STREAM("Timeout expired.");
        return false;
      }
      ROS_INFO_STREAM_THROTTLE(2,"Wait for updated planning scene...");
    }
    ros::Duration(0.2).sleep();
  }
  return true;
}

moveit_msgs::CollisionObject SceneObjectsManager::toCollisionObject( const std::string     &collisionObjID
                                                                                      , const std::string     &path_to_mesh
                                                                                      , const std::string     &reference_frame
                                                                                      , const tf::Pose        &pose
                                                                                      , const Eigen::Vector3d scale)
{
  std::shared_ptr<moveit_msgs::CollisionObject> collision_object(new moveit_msgs::CollisionObject );
  collision_object->id = collisionObjID;
  shapes::Mesh* m = shapes::createMeshFromResource ( path_to_mesh, scale );
  
  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape ( m, mesh_msg );
  mesh = boost::get<shape_msgs::Mesh> ( mesh_msg );
  
  collision_object->meshes.resize ( 1 );
  collision_object->mesh_poses.resize ( 1 );
  collision_object->meshes[0] = mesh;
  collision_object->header.frame_id = reference_frame;
  
  geometry_msgs::Pose pose_msg;
  tf::poseTFToMsg ( pose, pose_msg );
  
  collision_object->mesh_poses[0] = pose_msg;
  
  collision_object->meshes.push_back ( mesh );
  collision_object->mesh_poses.push_back ( collision_object->mesh_poses[0] );
  collision_object->operation = collision_object->ADD;
  
  return *collision_object;
}

bool SceneObjectsManager::addObjects( object_loader_msgs::addObjects::Request&  req
                                    , object_loader_msgs::addObjects::Response& res )
{
  std::vector< moveit_msgs::CollisionObject > objs;
  std::vector< moveit_msgs::ObjectColor     > colors;
  std::vector<std::string > v = planning_scene_interface_.getKnownObjectNames();
  
  for (auto obj : req.objects)
  {
    std::string id = obj.obj_id.data;
    
    int count = 0;
    id +="_";
    id +=std::to_string(count);
    
    if(std::count(v.begin(),v.end(),id))
    {
      while(std::count(v.begin(),v.end(),id))
      { 
        ROS_INFO_STREAM(id<<" already present, adding one");
        count ++;
        id = obj.obj_id.data;
        id +="_";
        id +=std::to_string(count);
      }
    }
    
    ROS_INFO_STREAM("adding "<<id);
    
    tf::Pose T_0_hc ;
    tf::poseMsgToTF( obj.pose.pose, T_0_hc );
    
    std::string path;
    if(!nh_.getParam(obj.obj_id.data,path))
    {
      ROS_ERROR_STREAM("param "<<nh_.getNamespace()<<"/"<< obj.obj_id.data <<" not found");
      res.success = false;
      return true;
    }      
    
    objs.push_back( toCollisionObject( id, path, obj.pose.header.frame_id, T_0_hc) );
    
    moveit_msgs::ObjectColor color; 
    color.id = id; color.color.r = 255; color.color.g = 255; color.color.b = 255; color.color.a = 1;
    
    colors.push_back(color);
  }
  
  if( !applyAndCheckPS( ros::NodeHandle()
                      , objs
                      , colors
                      , ros::Duration(10) ) )
  {
    ROS_FATAL_STREAM("Failed in uploading the collision objects");
    res.success = false;
  }
  else
  {
    ROS_INFO_STREAM("Ok! Objects loaded in the planning scene");
    
    //TODO comunicazione a inbound server
    
    res.success = true;
  }
  
  return true;
}

bool SceneObjectsManager::removeObjects( object_loader_msgs::removeObjects::Request&  req
                                      , object_loader_msgs::removeObjects::Response& res )
{
  std::vector<std::string > v;
  for (auto obj : req.obj_ids)
    v.push_back(obj.data);
  planning_scene_interface_.removeCollisionObjects ( v );
  res.success = true;
  return true;
}

bool SceneObjectsManager::resetScene( std_srvs::Trigger::Request&   req
                                    , std_srvs::Trigger::Response&  res )
{    
  std::vector<std::string > v = planning_scene_interface_.getKnownObjectNames();
  planning_scene_interface_.removeCollisionObjects ( v );
  
  ros::Duration(2.0).sleep();
  return (res.success = true); 
  
}

SceneObjectsManager::SceneObjectsManager( ros::NodeHandle nh )
: nh_  (nh)
{
  add_object_srv_    = nh_.advertiseService(ADD_OBJECT_SRV    , &SceneObjectsManager::addObjects   , this);
  remove_object_srv_ = nh_.advertiseService(REMOVE_OBJECT_SRV , &SceneObjectsManager::removeObjects, this);
  reset_srv_         = nh_.advertiseService(RESET_SCENE_SRV   , &SceneObjectsManager::resetScene   , this);
}
