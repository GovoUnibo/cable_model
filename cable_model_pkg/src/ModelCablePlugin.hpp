#ifndef MODEL_CABLE_PLUGIN
#define MODEL_CABLE_PLUGIN

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <chrono> 
#include <vector>
#include <string>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <cable_model.hpp>

#include <cable_model_pkg/GraspMsg.h>
// #include <cable_model_pkg/coordinates.h>  // No longer needed, using geometry_msgs/Pose

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
                simulation_start_time = std::chrono::steady_clock::now();
                // t0 = std::chrono::steady_clock::now();
            }

            ~CableModelPlugin(){
                ros::shutdown();
                std::cout << "cable model plugin stopped." << std::endl;
                
                if (forces_csv_file.is_open()) {
                    forces_csv_file.close();
                    std::cout << "File forces_data.csv chiuso correttamente." << std::endl;
                }
                if (position_csv_file.is_open()) {
                    position_csv_file.close();
                    std::cout << "File positions_data.csv chiuso correttamente." << std::endl;
                }

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
            void open_forces_csv(std::string);            
            void write_forces();
            void open_positions_csv(std::string);
            void write_positions();
            
            std::chrono::steady_clock::time_point simulation_start_time;
            
            ros::ServiceServer grasp_service;
            ros::NodeHandle ros_nh;
            std::ofstream forces_csv_file;
            std::ofstream position_csv_file;
            std::chrono::steady_clock::time_point start_time;

            bool callbackGraspServer(   cable_model_pkg::GraspMsg::Request &rqst,
                                        cable_model_pkg::GraspMsg::Response &res
                                    );
            

            void robot1PoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
            void robot2PoseCallback(const geometry_msgs::Pose::ConstPtr& msg);

            

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

            ros::Subscriber sub_mass0_coords;  // Subscriber for mass 0 coordinates (pos + rot)
            ros::Subscriber sub_massN_coords;  // Subscriber for mass N coordinates (pos + rot)

            ignition::math::Vector3d force_mass_0;
            // cable_model_pkg::coordinates force_mass_0_msg;  // Removed - using geometry_msgs/Pose now


            ignition::math::Vector3d robot1_pos, robot2_pos;
            ignition::math::Quaterniond robot1_rot, robot2_rot;  // Rotation for mass 0 and N

            ignition::math::Quaterniond computeMass0FinalRotation(const ignition::math::Quaterniond &robot_rot, const ignition::math::Quaterniond &curr_mass_rot);
            ignition::math::Quaterniond prev_robot1_rot_{0,0,0,1};
            bool prev1_initialized_{false};
            ignition::math::Quaterniond computeMassNFinalRotation(const ignition::math::Quaterniond &robot_rot, const ignition::math::Quaterniond &curr_mass_rot);
            ignition::math::Quaterniond prev_robot2_rot_{0,0,0,1};
            bool prev2_initialized_{false};


            bool isMass0Gripped = false;
            bool isMassNGripped = false;

           


    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(CableModelPlugin)
}


#endif