#include "agl_tracker.hh"

#include <iostream>
#include <fstream>
#include <string>
#include <cstring>

namespace gazebo
{

// Log file for the mission attributes
std::ofstream team_score_file;


void AglTracker::Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
{
  printf("[AglTracker] | Loading AGL Tracker Plugin...\n");

  // Used for visual insertions later on
  _world = _parent;

  // Load the Terrain as a heightmap
  physics::ModelPtr terrain_model = _world->ModelByName("terrain2d");
  physics::CollisionPtr terrain_collision = terrain_model->GetLink("link")->GetCollision("collision");
  _terrain_heightmap = boost::dynamic_pointer_cast<physics::HeightmapShape>(terrain_collision->GetShape());   
  
  // The world contains two models: Terrain and the iris_lmlidar
  _lm_iris_model = _world->ModelByName("iris_lmlidar");


  // Error checking 
  GZ_ASSERT(_lm_iris_model, "[AglTracker] | ERROR | LM IRIS Model has not been set up");
  GZ_ASSERT(terrain_model, "[AglTracker] | ERROR | Terrain has not been properly set up");

  // When the world updates, make sure the plugin gets to see what's happening
  _updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&AglTracker::OnUpdate, this));

  // Open log for recording trajectory and mission attributes
  team_score_file.open("team_name.csv");

  // Finished loading
}



void AglTracker::OnUpdate()
{
  ignition::math::Pose3d vehicle_pose = _lm_iris_model->WorldPose();
  double veh_x = vehicle_pose.Pos().X();
  double veh_y = vehicle_pose.Pos().Y();
  double veh_z = vehicle_pose.Pos().Z();

  static double agl = 0.0;

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
    double ground_truth_z =  _terrain_heightmap->GetHeight( index_x , index_y ); 
    agl = veh_z - ground_truth_z;

    // Use the AGL to create and insert an SDF model of a culinder to track what the accurate AGL would be
    InsertTrackingPillar(veh_x, veh_y, veh_z, ground_truth_z, TARGET_AGL);

    // Keep a running average of the AGL 
    double agl_err = std::fabs(agl - TARGET_AGL);

   
    _agl_avg       = (_agl_avg*_num_agl_samples + agl) / (_num_agl_samples + 1);
    _agl_avg_error = (_agl_avg_error*_num_agl_samples + agl_err) / (_num_agl_samples + 1);
    _num_agl_samples += 1;

    // Now that we've sampled and tracked this specific location, let's move on
    _last_tracked_x = veh_x;
    _last_tracked_y = veh_y;
  
    // Publish the known AGL
    PublishAGL(vehicle_pose, agl, ground_truth_z);
  }

  // Close the log file by attempting to detect a landing
  static int detected_landing_iters = 0.0;
  if (_running && agl < 0.5) 
  {
    detected_landing_iters++;
    //printf("AGL: %f| Landing? %d\n", agl, detected_landing_iters);
  
  } else if (_running && agl > 0.5){
    detected_landing_iters = 0.0;
  }

  // End it after a certain period of time
  if (_running && detected_landing_iters > 50) 
  {
    printf("AGL| Closing log file!\n");
    team_score_file.close();
    _running = false;
  }
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
                  <radius>0.1</radius>\
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



void AglTracker::PublishAGL(ignition::math::Pose3d veh_pose, double current_agl, double ground_z)
{
  // Print out a header only once and at the beginning
  if (!_running && current_agl > 0.5) 
  {
    // The structure of the logging
    std::string header;
    header  = "\n\nAGL|\t\tMission Attributes and Vehicle Status\n";
    header += "AGL|       ( X , Y , Z)     |    AGL   |!|     Score     |   Avg Error   \n";
    header += "AGL|========================|==========|!|===============|===============\n";
    
    std::cout << header;

    // Header for the file
    team_score_file << "Vehicle (X,Y,Z), Ground Truth Z, AGL, Score, Avg. Error\n";
  
    _running = !_running;
  }

  double x = veh_pose.Pos().X();
  double y = veh_pose.Pos().Y();
  double z = veh_pose.Pos().Z();

  // Print out the current status
  common::Time timestamp = _world->RealTime();

  char score_chars[256];
  std::sprintf(score_chars,"AGL|   ( %2.2f, %2.2f, %2.2f ) |  %.3f  |!|      %.5f      |   %.5f   \n",\
              x, y, z, current_agl, _agl_avg, _agl_avg_error);
  
  std::cout << std::string(score_chars);

  // Write to a .csv
  std::sprintf(score_chars, "%f, %f, %f, %f, %f, %f, %f\n", \
                              x, y, z, ground_z, current_agl, _agl_avg, _agl_avg_error);
  team_score_file << std::string(score_chars);
}

} // namespace gazebo
