#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include "gazebo/physics/CylinderShape.hh"

namespace gazebo
{
class AglTracker : public WorldPlugin
{
  public: 
  
  void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    printf("Loading AGL Tracker Plugin...\n");

    // Used for visual insertions later on
    _world = parent;

    // No takeoff detected yet, wait until the next update
    _running = false;
    _num_agl_samples = 0;
    _agl_running_error = 0.0;

    // The world contains two models: Terrain and the iris_lmlidar
    _lm_iris_model = _parent->getModel("iris_lmlidar");
    _last_tracked_x = 0.0;  // Vehicle, by default starts off at 0,0 relative to world terrain
    _last_tracked_y = 0.0;
    

    // Load the Terrain as a heightmap
    physics::ModelPtr terrain_model = _parent->getModel("terrain2d")
    physics::CollisionPtr collision = terrain_model->GetLink("link")->GetCollision("collision");
    _terrain_heightmap = boost::dynamic_pointer_cast<physics::HeightmapShape>(collision->GetShape());   
    _terrain_heightmap->subSampling(1); // Samples every meter

    // When the world updates, make sure the plugin gets to see what's happening
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&AglTracker::OnUpdate, this));

    printf("... Finished loading AGL Tracker Plugin. Enjoy!");
  }

  /** OnUpdate
   * @brief Place a tracker at intervals to display the vehicle's progress
   * 
   * Each iteration, compare the known vehicle's pose to the previous pose, 
   * If there is a large enough horizontal offset then place a tracker on the correct AGL
   * Compare the vehicle's known pose to the heightmap's alttitude at the given pose's (x,y), and find the offset
   * 
   * TODO: Find elapsed time, but needs to know about takeoff and landing. Z measurements?
   */
  void OnUpdate()
  {
    ignition::math::Pose3d vehicle_pose = _lm_lidar_model->WorldPose();
    double veh_x = vehicle_pose.Pos().X();
    double veh_y = vehicle_pose.Pos().Y();
    double veh_z = vehicle_pose.Pos().Z();

    // TODO: Calculate if simulation is 

    // Compare current pose with last tracked pose
    double relative_dist = (veh_x - _last_tracked_x)*(veh_x - _last_tracked_x) +
                           (veh_y - _last_tracked_y)*(veh_y - _last_tracked_y);

    if (relative_dist >= TRACKING_DIST*TRACKING_DIST) // I hate square roots
    {
      //Sample Heightmap AGL at current world (X, Y) to the HeightmapShape (index_x, index_y)
      math::Vector3 hm_size = _terrain_heightmap->GetSize(); 
      math::Vector2i hm_vc  = _terrain_heightmap->GetVertexCount();
      int index_x = (  (  (veh_x + hm_size.x/2)/hm_size.x ) * hm_vc.x - 1 ) ;
      int index_y = (  ( (-veh_y + hm_size.y/2)/hm_size.y ) * hm_vc.y - 1 ) ;

      //  getting the height :
      double ground_truth_z =  _terrain_heightmap->GetHeight( index_x , index_y ); 
      double agl = veh_z - ground_truth_z;

      // Use the AGL to create and insert an SDF model of a culinder to track what the accurate AGL would be
      // InsertTrackingPillar(veh_x, veh_y, veh_z, ground_truth_z + TARGET_AGL);

      // Keep a running average of the AGL 
      _agl_running_error += (agl - TARGET_AGL);
      _num_agl_samples++;
      _agl_avg_error = _agl_running_error / _num_agl_samples;

      // Now that we've sampled and tracked this specific location, let's move on
      _last_tracked_x = veh_x;
      _last_tracked_y = veh_y;
    }

    /** InsertTrackingPillar
     * @brief After measuring vehicle AGL, insert visual pillar on its accuracy
     * The pillar will start at the vehicle's location and will display the difference between the target Z vs. current Z
     * @param center_x Center of vehicle along X, in gazebo units
     * @param center_y Center of vehicle along Y
     * @param center_z Center of vehicle along Z
     * @param target_agl The vehicle's target AGL
     */
    void InsertTrackingPillar(double center_x, double center_y, double center_z, double target_agl)
    {
      sdf::SDF example_sphere;
      example_sphere.SetFromString(
        "<sdf version ='1.4'>\
            <model name ='sphere'>\
              <pose>1 0 1 0 0 0</pose>\
              <link name ='link'>\
                <pose>0 0 .5 0 0 0</pose>\
                <visual name ='visual'>\
                  <geometry>\
                    <sphere><radius>0.5</radius></sphere>\
                  </geometry>\
                </visual>\
              </link>\
            </model>\
          </sdf>");
      // Demonstrate using a custom model name.
      sdf::ElementPtr ex_model = example_sphere.Root()->GetElement("model");
      ex_model->GetAttribute("name")->SetFromString("unique_sphere");
      _world->InsertModelSDF(example_sphere);

    }

    // Once goal is reached, perform all scoring calculations
    // TODO: Track vehicle pose until == goal
  }


  private:
  // Scoring attributes
  const double TARGET_AGL = 10; // in meters
  int _num_agl_samples;         // Number of AGL samples taken
  double _agl_running_error;     // Sum of AGL error throughout trajectory
  double _agl_avg_error;        // Accuracy of overall track denoted by low error. Running average

  common::Time _start_sim_time; // Time where vehicle is detected to have moved (i.e height is nonzero)
  common::Time _end_sim_time;   // Time where vehicle has reached the goal
  double _elapsed_time;         // in seconds

  // Models found in the world
  physics::WorldPtr _world;                      // Pointer to the world.
  physics::ModelPtr _lm_iris_model;              // Pointer to the Iris model. Used for world pose tracking
  phyics::HeightMapShapePtr _terrain_heightmap;  // Pointer to the Terrain model. Used for heightmap lookups

  // Model state variables
  bool _running;           // False to True implies takeoff detected, True to False implies goal has been reached
  // Denote the correct locations for the given AGL 
  double _last_tracked_x;  // Previous X coord to be tracked
  double _last_tracked_y;  // Previous Y coord to be tracked
  const double TRACKING_DIST = 2.0;   // Track the vehicle and sample it's AGL every 2 meters horizontally in any direction

  // Physical parameters of the model during the run
  double _max_horz_speed;     // Largest linear velocity along (x,Y), find vector length
  double _min_horz_speed;     // Smallest linear velocity along (X,Y), find vector length

  double _max_vert_speed;     // Largest linear velocity aloing (Z)
  double _min_vert_speed;     // Smallest linear velocity along (Z)

  // TODO: Publish a message or a file at the end containing the accuracy

}; // class AglTracker

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(AglTracker)
} // namespace gazebo



 // // Option 1: Insert model from file via function call.
    // // The filename must be in the GAZEBO_MODEL_PATH environment variable.
    // _parent->InsertModelFile("model://box");

    // // Option 2: Insert model from string via function call.
    // // Insert a sphere model from string
    // sdf::SDF sphereSDF;
    // sphereSDF.SetFromString(
    //    "<sdf version ='1.4'>\
    //       <model name ='sphere'>\
    //         <pose>1 0 0 0 0 0</pose>\
    //         <link name ='link'>\
    //           <pose>0 0 .5 0 0 0</pose>\
    //           <collision name ='collision'>\
    //             <geometry>\
    //               <sphere><radius>0.5</radius></sphere>\
    //             </geometry>\
    //           </collision>\
    //           <visual name ='visual'>\
    //             <geometry>\
    //               <sphere><radius>0.5</radius></sphere>\
    //             </geometry>\
    //           </visual>\
    //         </link>\
    //       </model>\
    //     </sdf>");
    // // Demonstrate using a custom model name.
    // sdf::ElementPtr model = sphereSDF.Root()->GetElement("model");
    // model->GetAttribute("name")->SetFromString("unique_sphere");
    // _parent->InsertModelSDF(sphereSDF);

    // // Option 3: Insert model from file via message passing.
    // {
    //   // Create a new transport node
    //   transport::NodePtr node(new transport::Node());

    //   // Initialize the node with the world name
    //   node->Init(_parent->Name());

    //   // Create a publisher on the ~/factory topic
    //   transport::PublisherPtr factoryPub =
    //   node->Advertise<msgs::Factory>("~/factory");

    //   // Create the message
    //   msgs::Factory msg;

    //   // Model file to load
    //   msg.set_sdf_filename("model://cylinder");

    //   // Pose to initialize the model to
    //   msgs::Set(msg.mutable_pose(),
    //       ignition::math::Pose3d(
    //       ignition::math::Vector3d(1, -2, 0),
    //       ignition::math::Quaterniond(0, 0, 0)));

    //   // Send the message
    //   factoryPub->Publish(msg);


    // }
  