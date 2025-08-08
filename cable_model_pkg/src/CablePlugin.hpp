#ifndef OPTIMIZED_CABLE_PLUGIN_HPP
#define OPTIMIZED_CABLE_PLUGIN_HPP

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <ignition/math/Vector3.hh>

#include <memory>
#include <string>
#include <vector>

#include "cable_model.hpp"
#include "cable_model/sdf_sphere.hpp"
#include "cable_model/sdf_cylinder.hpp"

#include "cable_model_pkg/GraspMsg.h"
#include <colors.hpp>
#include <geometry_msgs/Pose.h>


namespace gazebo
{
    class OptimizedCablePlugin : public WorldPlugin
    {
    public:
        // Constructor and destructor
        OptimizedCablePlugin() {}
        virtual ~OptimizedCablePlugin() {}

        // WorldPlugin Load function
        void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf_world) override;

        // Update function for cable control
        void OnUpdate();

        // ROS callback functions
        void robot1PoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
        void robot2PoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
        bool callbackGraspServer(cable_model_pkg::GraspMsg::Request &req, cable_model_pkg::GraspMsg::Response &res);

    private:
        // World pointer
        physics::WorldPtr world;

        // Cable creation parameters
        float width, length;
        float pos_x = 0.0f, pos_y = 0.0f, pos_z = 0.0f;
        float rot_x = 0.0f, rot_y = 0.0f, rot_z = 0.0f;
        float mass , damping, torsion_damper, young_modulus, poisson_ratio;
        bool gravity = true;
        float resolution;
        int num_particles, num_particle_links;
        std::string prefix_mass_names = "m";
        std::string cable_type = "sphere";

        double baseDt;      // dt di default
        double forceThresh; // soglia di forza
        double minScale;    // fattore minimo (es. 0.1)
        double maxScale;    // fattore massimo (es. 2.0)
        
        // Step size optimization variables
        int stepOptimizationCounter;
        int stepOptimizationInterval; // ogni quanti frame fare ottimizzazione
        double lastOptimizedScale;

        // SDF builders
        sdf_sphere::SphereSdf sphere_sdf;
        sdf_cylinder::CylinderSdf cylinder_sdf;

        // Cable control components
        std::shared_ptr<cable_utils::Cable> cable;
        physics::ModelPtr cable_model;
        bool cable_model_found = false;

        // ROS components
        ros::NodeHandle ros_nh;
        ros::Subscriber sub_mass0_coords;
        ros::Subscriber sub_massN_coords;
        ros::ServiceServer graspServer;
        std::string model_name;

        // Robot states
        ignition::math::Vector3d robot1_pos = {0.0, 0.0, 0.0};
        ignition::math::Vector3d robot2_pos = {0.0, 0.0, 0.0};
        ignition::math::Quaterniond robot1_rot = {1.0, 0.0, 0.0, 0.0};
        ignition::math::Quaterniond robot2_rot = {1.0, 0.0, 0.0, 0.0};

        bool isMass0Gripped = false;
        bool isMassNGripped = false;

        // Rotation tracking for incremental updates
        bool prev1_initialized_ = false;
        bool prev2_initialized_ = false;
        ignition::math::Quaterniond prev_robot1_rot_;
        ignition::math::Quaterniond prev_robot2_rot_;

        // Update connection
        event::ConnectionPtr updateConnection;

        // Helper functions
        void readCableParameters(sdf::ElementPtr _sdf);
        void cableInfoMsg();
        void checkROSInitializzation();
        void pushParamToParamServer();
        void setupCableControl();
        void setupCableModel();

        ignition::math::Quaterniond computeMass0FinalRotation(
            const ignition::math::Quaterniond &robot_rot,
            const ignition::math::Quaterniond &curr_mass_rot);
        
        ignition::math::Quaterniond computeMassNFinalRotation(
            const ignition::math::Quaterniond &robot_rot,
            const ignition::math::Quaterniond &curr_mass_rot);
    };

// Note: Plugin registration is done in the .cpp file
}

#endif // OPTIMIZED_CABLE_PLUGIN_HPP
