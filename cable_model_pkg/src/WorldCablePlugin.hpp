#ifndef WORLD_CABLE_PLUGIN
#define WORLD_CABLE_PLUGIN

#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "ros/ros.h"
#include "ros/package.h"
#include <geometry_msgs/Pose.h>
#include <string>

#include <sdf_sphere.hpp>

using namespace gazebo;
using namespace sdf_sphere;




enum _FATAL_ERROR_MSG_TYPE {PARAMETER_MISSING, ROS_ERROR};




class CableSpawner : public WorldPlugin, private SphereSdf
{
    private:
        physics::WorldPtr world;
        float width = 0.0, length = 0.0;
        float pos_x = 0.0, pos_y = 0.0, pos_z = 0.0;
        float rot_x = 0.0, rot_y = 0.0, rot_z = 0.0;
        float mass = 0.0, damping = 0.0, young_modulus = 0.0, poisson_ratio = 0.0;
        bool gravity = true;
        float resolution = 1;
        int num_particles, num_particle_links;
        std::string prefix_mass_names;

        void checkROSInitializzation();


        
        void cableInfoMsg();


        void readCableParameters(sdf::ElementPtr _sdf);
        void pushParamToParamServer();
        ros::NodeHandle ros_nh;

      public: 
        CableSpawner();
        ~CableSpawner();


        void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf_world);

};

GZ_REGISTER_WORLD_PLUGIN(CableSpawner)



#endif