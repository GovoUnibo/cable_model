#include "ModelCablePlugin.hpp"

using namespace gazebo;
using namespace std;
using namespace cable_utils;

// void GripInfoCallback(const std_msgs::Bool::ConstPtr& msg)
// {
//   setFirstMassGripped(msg->data);
// }


void CableModelPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){


    // Store the pointer to the model
    this->model = _parent;

    double poisson_ratio, young_modulus, damping; 

    std::vector<double> pos;
    std::vector<double> rot;
    // ros::NodeHandle ros_nh;
    ros_nh.getParam("/cable/width", this->width);
    ros_nh.getParam("/cable/length", this->length);
    ros_nh.getParam("/cable/resolution", this->resolution);
    ros_nh.getParam("/cable/rotation", rot);
    ros_nh.getParam("/cable/position", pos);
    ros_nh.getParam("/cable/mass", this->mass);
    ros_nh.getParam("/cable/damping", damping);
    ros_nh.getParam("/cable/gravity", this->gravity);
    ros_nh.getParam("/cable/num_particles", this->num_particles);
    ros_nh.getParam("/cable/prefix_mass_names", this->prefix_mass_names);
    ros_nh.getParam("/cable/poisson_ratio", poisson_ratio);
    ros_nh.getParam("/cable/young_modulus", young_modulus);

    this->pos_x = pos[0];
    this->pos_y = pos[1];
    this->pos_z = pos[2];
    this->rot_x = rot[0];
    this->rot_y = rot[1];
    this->rot_z = rot[2];

    cout << poisson_ratio << endl;
    cout << young_modulus << endl;
    
    // cout <<this->model->RelativePose().Pos().X() << endl; //ignition::math::Pose3d
    cable = std::make_shared<cable_utils::Cable>(this->model, this->pos_x, this->pos_y, this->pos_z, this-> mass, this->width, this->length, this->num_particles, this->prefix_mass_names);

    cable->useGravity(this->gravity);
    cable->setDamperCoef(damping);
    cable->setYoungModulus(young_modulus);
    cable->setPoissonRatio(poisson_ratio);
    // cout << link->RelativePose().Pos().X() << endl;


    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&CableModelPlugin::OnUpdate, this));

  if (!ros::isInitialized()) {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "cable_plugin_client",
        ros::init_options::NoSigintHandler);
  }
  publish_force_mass_0 = ros_nh.advertise<cable_model::coordinates>("/force_mass_0", 1);
}

bool CableModelPlugin::callbackGraspServer(cable_model::GraspMsg::Request &rqst, cable_model::GraspMsg::Response &res){
                            cable->setFirstMassGrasped(rqst.set_mass0_grasped);
                            cable->setLastMassGrasped(rqst.set_massN_grasped);
                            cable->setFirstMassFixed(rqst.set_mass0_fixed);
                            cable->setLastMassFixed(rqst.set_massN_fixed);
                            return true;
                        }
                        
void CableModelPlugin::OnUpdate(){
    // auto t = std::chrono::steady_clock::now();
    
    // force_mass_0 = cable->getForceWrtWorld(0);

    // force_mass_0_msg.x = force_mass_0.X();
    // force_mass_0_msg.y = force_mass_0.Y();
    // force_mass_0_msg.z = force_mass_0.Z();

    // publish_force_mass_0.publish(force_mass_0_msg);

    // // std::cout << "Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(t - t0).count() << " ms" << std::endl;
    // t0 = t;
 

    cable->updateModel();






}