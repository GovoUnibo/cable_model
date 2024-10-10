#ifndef MODEL_CABLE_PLUGIN
#define MODEL_CABLE_PLUGIN


#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <iostream>
#include <chrono>  
#include <vector>
#include <string>
#include <cable_model.hpp>

#include <cable_model_pkg/GraspMsg.h>
#include <cable_model_pkg/coordinates.h>

// #include <std_msgs/Bool.h>

using namespace std;

namespace gazebo
{
    class CableModelPlugin : public ModelPlugin
    {
        public:

            CableModelPlugin() : ModelPlugin(){
                std::cout << "Starting cable model plugin..." << std::endl;
                grasp_service = ros_nh.advertiseService("set_cable_grasp", &CableModelPlugin::callbackGraspServer, this);
                // t0 = std::chrono::steady_clock::now();
            }

            ~CableModelPlugin(){
                ros::shutdown();
                std::cout << "cable model plugin stopped." << std::endl;
            }

            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

            // Called by the world update start event
            public: void OnUpdate();
        
            // void setFirstMassGripped(bool value){
            //     isFirstMassGripped = value;
            // }

            // void GripInfoCallback(const std_msgs::Bool::ConstPtr& msg);

        // Pointer to the model
        private: 
            ros::ServiceServer grasp_service;
            ros::NodeHandle ros_nh;

            bool callbackGraspServer(   cable_model_pkg::GraspMsg::Request &rqst,
                                        cable_model_pkg::GraspMsg::Response &res
                                    );


            physics::ModelPtr model;
            std::chrono::steady_clock::time_point t0;
            // Pointer to the update event connection
            event::ConnectionPtr updateConnection;
            
            float width = 0.0, length = 0.0;
            float pos_x = 0.0, pos_y = 0.0, pos_z = 0.0;
            float rot_x = 0.0, rot_y = 0.0, rot_z = 0.0;
            float mass = 0.0; //stiffness = 0.0, damping = 0.0, bending = 0.0, twisting = 0.0;
            bool gravity = true;
            float resolution = 1;
            int num_particles;
            std::string prefix_mass_names;

            using CablePtr = std::shared_ptr<cable_utils::Cable>;
            CablePtr cable;

            ros::Publisher publish_force_mass_0;

            ignition::math::Vector3d force_mass_0;
            cable_model_pkg::coordinates force_mass_0_msg;

    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(CableModelPlugin)
}


#endif