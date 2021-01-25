#include "agl_tracker.hh"

namespace gazebo
{

void AglTracker::Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
{
  printf("Loading AGL Tracker Plugin...\n");

  // Used for visual insertions later on
  _world = _parent;

  // Load the Terrain as a heightmap
  physics::ModelPtr terrain_model = _world->ModelByName("terrain2d");
  physics::CollisionPtr terrain_collision = terrain_model->GetLink("link")->GetCollision("collision");
  _terrain_heightmap = boost::dynamic_pointer_cast<physics::HeightmapShape>(terrain_collision->GetShape());   

  if (!terrain_model)
    printf("Terrain has not been set up\n");
  // The world contains two models: Terrain and the iris_lmlidar
  _lm_iris_model = _world->ModelByName("iris_lmlidar");

  if(!_lm_iris_model)
     printf("LM IRIS Model has not been set up\n");   

  // When the world updates, make sure the plugin gets to see what's happening
  _updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&AglTracker::OnUpdate, this));

  // Advertise the topic
  // It will hold the vehicle's location, the AGL at that location
  // and the entire heightmap represented as a flattened 2D 
  _publisher_node = transport::NodePtr(new transport::Node());
  _publisher_node->Init(_world->Name());
  _agl_publisher = _publisher_node->Advertise<gazebo::msgs::AglDebug>(_default_agl_topic);

  // Set the initial set of variables for uORB
  /*
  _veh_status_sub = orb_subscribe(ORB_ID(vehicle_status));
  _veh_status_poll[0].fd = sensor_sub_fd;
  _veh_status_poll[0].events = POLLIN;
  */

  printf("... Finished loading AGL Tracker Plugin. Enjoy!");
}



void AglTracker::OnUpdate()
{
  ignition::math::Pose3d vehicle_pose = _lm_iris_model->WorldPose();
  double veh_x = vehicle_pose.Pos().X();
  double veh_y = vehicle_pose.Pos().Y();
  double veh_z = vehicle_pose.Pos().Z();

  double ground_truth_z = 0.0;

  // Compare current pose with last tracked pose
  double relative_dist = (veh_x - _last_tracked_x)*(veh_x - _last_tracked_x) +
                         (veh_y - _last_tracked_y)*(veh_y - _last_tracked_y);

  // Only bother to update any visual guidance markers 
  if (relative_dist >= TRACKING_DIST*TRACKING_DIST) // I hate square roots
  {
    //Sample Heightmap AGL at current world (X, Y) to the HeightmapShape (index_x, index_y)
    ignition::math::Vector3d hm_size = _terrain_heightmap->Size(); 
    ignition::math::Vector2i hm_vc  = _terrain_heightmap->VertexCount();
    int index_x = (  (  (veh_x + hm_size.X()/2)/hm_size.X() ) * hm_vc.X() - 1 ) ;
    int index_y = (  ( (-veh_y + hm_size.Y()/2)/hm_size.Y() ) * hm_vc.Y() - 1 ) ;

    //  getting the height :
    ground_truth_z =  _terrain_heightmap->GetHeight( index_x , index_y ); 
    double agl     = veh_z - ground_truth_z;

    // Use the AGL to create and insert an SDF model of a culinder to track what the accurate AGL would be
    InsertTrackingPillar(veh_x, veh_y, veh_z, ground_truth_z, TARGET_AGL);

    // Keep a running average of the AGL 
    double agl_err = (agl - TARGET_AGL);
    _agl_avg_error = (_agl_avg_error + agl_err) / ++_num_agl_samples;

    // Now that we've sampled and tracked this specific location, let's move on
    _last_tracked_x = veh_x;
    _last_tracked_y = veh_y;
  }

  // Publish the known AGL
  PublishAGL(vehicle_pose, veh_z - ground_truth_z, _agl_avg);

  // Once goal is reached, perform all scoring calculations
  // TODO: Track vehicle pose until == goal
}



void AglTracker::InsertTrackingPillar(double center_x, double center_y, double center_z, double terrain_ground_truth, double target_agl)
{
  // Only display the error between vehicle and target AGL
  double displ_agl_err = (center_z + terrain_ground_truth + target_agl)/2.0;

  // Create a string to describe the cylinder
  std::stringstream cylinder_sdf_str;
  cylinder_sdf_str << ""\
      "<sdf version ='1.4'>\
        <model name ='cylinder'>\
          <static>true</static>\
          <link name ='link'>\
            <pose>" << center_x << " " << center_y << " " << displ_agl_err << " 0 0 0</pose>\
            <visual name ='visual'>\
              <geometry>\
                <cylinder>\
                  <radius>0.3</radius>\
                  <length>" << center_z - target_agl << "</length>\
                </cylinder>\
              </geometry>\
            </visual>\
          </link>\
        </model>\
      </sdf>";

  // Based on the string above, create a cylinder
  sdf::SDF agl_cylinder;
  agl_cylinder.SetFromString(cylinder_sdf_str.str());
  _world->InsertModelSDF(agl_cylinder);
}



void AglTracker::PublishAGL(ignition::math::Pose3d veh_pose, double current_agl, double avg_agl)
{
  // Fill the AGL Tracker message up with updated stats
  _agl_msg.set_elapsed_time(_elapsed_time);
  _agl_msg.set_current_agl(current_agl);
  _agl_msg.set_average_agl(_agl_avg);

  // Pose to initialize the model to
  msgs::Set(_agl_msg.mutable_vehicle_pose(),
      ignition::math::Pose3d(
        ignition::math::Vector3d(veh_pose.Pos().X(), veh_pose.Pos().Y(), veh_pose.Pos().Z()),
        ignition::math::Quaterniond(veh_pose.Rot().X(), veh_pose.Rot().Y(), veh_pose.Rot().Z())));

  // Now publish the AGL message
  _agl_publisher->Publish(_agl_msg);
}

// uint AglTracker::VehicleNavState()
// {
//   // wait for sensor update of 1 file descriptor for 1000 ms (1 second) 
// 	int poll_ret = px4_poll(_veh_status_poll, 1, 1000);

//   // handle the poll result */
//   if (poll_ret == 0) {
//     // this means none of our providers is giving us data 
//     PX4_ERR("AglTracker Plugin | Didn't gets no 'vehicle state' within a second");

//   } else if (poll_ret < 0) {
//     // use a counter to prevent flooding (and slowing us down) 
//     PX4_ERR("AglTracker Plugin | ERROR return value from poll(): %d", poll_ret);

//   } else {
//     // Good return, now process it
//     if (_veh_status_poll[0].revents & POLLIN) 
//     {
//       /* obtained data for the first file descriptor */
//       struct vehicle_status_s raw_veh_status;

//       /* copy vehicle status raw data into local buffer */
//       orb_copy(ORB_ID(vehicle_status), sensor_sub_fd, &raw_vehicle_status);

//       // Return the nav state
//       return raw_veh_status.nav_state;
//     }

//     /* there could be more file descriptors here, in the form like:
//       * if (fds[1..n].revents & POLLIN) {}
//       */
//   }

//   // Useless nav state
//   return NAVIGATION_STATE_UNUSED;
// }

} // namespace gazebo
