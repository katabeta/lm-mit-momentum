#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector2.hh>

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/gazebo.hh"

#include "gazebo/physics/CylinderShape.hh"
#include "gazebo/physics/HeightmapShape.hh"
#include "gazebo/physics/Model.hh"

namespace gazebo
{

class AglTracker : public WorldPlugin
{
  public: 

  AglTracker() : WorldPlugin()
  {
    // No takeoff detected yet, wait until the next update
    _running = false;

    _num_agl_samples = 0;
    _agl_avg = TARGET_AGL;
    _agl_avg_error = 0.0;

    _last_tracked_x = 0.0;  // Vehicle, by default starts off at 0,0 relative to world terrain
    _last_tracked_y = 0.0;

    _terrain_min_height = 0.0; // Minimum z-value of heightmap of terrain
    _terrain_max_height = 0.0; // Maximum z-value of heightmap of terrain

    // Possible information for further debug
    _min_vert_speed = _min_horz_speed = 100.0;
    _max_vert_speed = _max_horz_speed = 0.0; 
  }
  
  void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/);

  /** 
   * @brief Place a tracker at intervals to display the vehicle's progress
   * Each iteration, compare the known vehicle's pose to the previous pose, 
   * If there is a large enough horizontal offset then place a tracker on the correct AGL
   * Compare the vehicle's known pose to the heightmap's alttitude at the given pose's (x,y), and find the offset
   * 
   * TODO: Find elapsed time, but needs to know about takeoff and landing. Z measurements?
   */
  void OnUpdate();

  /** 
   * @brief After measuring vehicle AGL, insert visual pillar on its accuracy
   * The pillar will start at the vehicle's location and will display the difference between the target Z vs. current Z
   * @param center_x Center of vehicle along X, in gazebo units
   * @param center_y Center of vehicle along Y
   * @param center_z Center of vehicle along Z
   * @param ground_truth_z Altitude of the ground according to heightmap
   * @param target_agl The vehicle's target AGL
   */
  void InsertTrackingPillar(double center_x, double center_y, double center_z, double ground_truth_z, double target_agl);


  /**
   * @brief Print out Team status "
   * 
   * @param veh_pose 
   * @param current_agl 
   * @param ground_z
   */
void PublishAGL(ignition::math::Pose3d veh_pose, double current_agl, double ground_z);

  /**
   * @brief Subscribe to Vehicle Status via uORB 
   * The navigation state is a part of the message, and the state will be leveraged
   * to detec when the vehicle is taking off and landing
   */
  uint VehicleNavState();
  

  private:

  // Scoring attributes
  const double TARGET_AGL = 3.0; // in meters, or gazebo units
  int _num_agl_samples;          // Number of AGL samples taken
  double _agl_avg;               // Average of measured AGL throughout trajectory
  double _agl_avg_error;         // Accuracy of overall track denoted by low error. Sum of absolute (error)

  common::Time _start_sim_time; // Time where vehicle is detected to have moved (i.e height is nonzero)
  common::Time _end_sim_time;   // Time where vehicle has reached the goal
  double _elapsed_time;         // in seconds

  // Models found in the parent world
  physics::WorldPtr _world;                      // Pointer to the world.
  physics::ModelPtr _lm_iris_model;              // Pointer to the Iris model. Used for world pose tracking
  physics::HeightmapShapePtr _terrain_heightmap;  // Pointer to the Terrain model. Used for heightmap lookups

  // Event handling
  event::ConnectionPtr _updateConnection; // Callback event whenever the world begins its update step

  // Model state variables
  bool _running;           // False to True implies takeoff detected, True to False implies goal has been reached
  
  // Denote the correct locations for the given AGL 
  double _last_tracked_x;  // Previous X coord to be tracked
  double _last_tracked_y;  // Previous Y coord to be tracked
  const double TRACKING_DIST = 1.0;   // Track the vehicle and sample it's AGL every 2 meters horizontally in any direction
  double _terrain_min_height; // Minimum  
  double _terrain_max_height; 

  // Physical parameters of the model during the run
  double _max_horz_speed;     // Largest linear velocity along (x,Y), find vector length
  double _min_horz_speed;     // Smallest linear velocity along (X,Y), find vector length
  double _max_vert_speed;     // Largest linear velocity aloing (Z)
  double _min_vert_speed;     // Smallest linear velocity along (Z)

  // Subscribe to the vehicle status
  int _veh_status_sub;
  // px4_pollfd_struct_t _veh_status_poll[];

}; // class AglTracker

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(AglTracker)
} // namespace gazebo