#include "ModelCablePlugin.hpp"
#include <geometry_msgs/Pose.h>

using namespace gazebo;
using namespace std;
using namespace cable_utils;

// void GripInfoCallback(const std_msgs::Bool::ConstPtr& msg)
// {
//   setFirstMassGripped(msg->data);
// }
#define RESET "\033[0m"
#define ORANGE "\033[38;5;214m"

void CableModelPlugin::open_forces_csv(std::string folder_path){
    // Apri file CSV per scrivere le forze
    this->start_time = std::chrono::steady_clock::now();
    std::string file_path = folder_path + "forces_data.csv";
    forces_csv_file.open(file_path, std::ios::out);
    if (!forces_csv_file.is_open()) {
        std::cerr << "Errore: impossibile aprire il file forces_data.csv!" << std::endl;
        exit(0);  // Puoi decidere di uscire dalla funzione o lanciare un'eccezione
    }
    else {
        std::cout << ORANGE << "File forces_data.csv aperto correttamente." << RESET << std::endl;
    }

    forces_csv_file << "time";  // Prima colonna: tempo
    for (int i = 0; i < this->num_particles; ++i) {
        forces_csv_file << ",force_magnitude_mass_" << i;  // Modifica per scrivere il modulo della forza
    }
    forces_csv_file << "\n";
}


void CableModelPlugin::open_positions_csv(std::string folder_path) {
    // Apri file CSV per scrivere le posizioni
    this->start_time = std::chrono::steady_clock::now();
    std::string file_path= folder_path  + "positions_data.csv";
    position_csv_file.open(file_path, std::ios::out);
    if (!position_csv_file.is_open()) {
        std::cerr << "Errore: impossibile aprire il file positions_data.csv!" << std::endl;
        exit(0);  // Puoi decidere di uscire dalla funzione o lanciare un'eccezione
    } else {
        std::cout << ORANGE << "File positions_data.csv aperto correttamente." << RESET << std::endl;
    }

    // Scrivi le intestazioni per il tempo, le posizioni x e z
    position_csv_file << "time";  // Prima colonna: tempo
    for (int i = 0; i < this->num_particles; ++i) {
        position_csv_file << ",position_x_" << i << ",position_z_" << i;  // Posizioni x e z per ogni particella
    }
    position_csv_file << "\n";
}


void CableModelPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){


    // Store the pointer to the model
    this->model = _parent;

    double poisson_ratio, young_modulus, damping, torsion_damper;

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
    ros_nh.getParam("/cable/torsion_damper", torsion_damper);
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
    cable->setTorsionDamperCoef(0);
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
    std::string package_path = ros::package::getPath("cable_model_pkg");


    this->sub_mass0_coords = ros_nh.subscribe("/mass0/pose", 1, &CableModelPlugin::robot1PoseCallback, this);
    this->sub_massN_coords = ros_nh.subscribe("/massN/pose", 1, &CableModelPlugin::robot2PoseCallback, this);


    this->simulation_start_time = std::chrono::steady_clock::now();
}



// Callback per coordinate complete massa 0 (posizione + rotazione)
void CableModelPlugin::robot1PoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    this->robot1_pos = {msg->position.x,
                      msg->position.y,
                      msg->position.z};
  this->robot1_rot = {msg->orientation.w,
                      msg->orientation.x,
                      msg->orientation.y,
                      msg->orientation.z};
}

// Callback per coordinate complete massa N (posizione + rotazione)
void CableModelPlugin::robot2PoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    this->robot2_pos = {msg->position.x,
                      msg->position.y,
                      msg->position.z};
    this->robot2_rot = {msg->orientation.w,
                      msg->orientation.x,
                      msg->orientation.y,
                      msg->orientation.z};
}

bool CableModelPlugin::callbackGraspServer(cable_model_pkg::GraspMsg::Request  &req,cable_model_pkg::GraspMsg::Response &res){
  this->isMass0Gripped = req.grasp_mass_0;
  this->isMassNGripped = req.grasp_mass_N;
  return true;
}

                        
void CableModelPlugin::OnUpdate(){

    if (this->isMass0Gripped) {
        auto link0   = this->cable->getLink(0);
        auto curr_q0 = link0->WorldPose().Rot();
        auto new_q0  = this->computeMass0FinalRotation(this->robot1_rot, curr_q0);
        this->cable->setMassPosition(0, this->robot1_pos, new_q0);
    }

    // MASSA N
    if (this->isMassNGripped) {
        int idxN    = this->num_particles - 1;
        auto linkN  = this->cable->getLink(idxN);
        auto curr_qN = linkN->WorldPose().Rot();
        auto new_qN  = this->computeMassNFinalRotation(this->robot2_rot, curr_qN);
        this->cable->setMassPosition(idxN, this->robot2_pos, new_qN);
    }


    

    cable->alignMassFrames();
    cable->updateModel();

}

void CableModelPlugin::write_forces(){
    // Ottieni il tempo corrente
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();

    // Scrivi il tempo corrente sul file
    forces_csv_file << elapsed;

    // Per ogni massa, ottieni la forza e calcola il modulo
    for (int i = 0; i < this->num_particles; ++i) {
        auto force = cable->getForceWrtWorld(i);
        
        // Calcolo del modulo della forza
        double force_magnitude = sqrt(force.X() * force.X() + force.Y() * force.Y() + force.Z() * force.Z());

        // Scrivi il modulo della forza sul CSV
        forces_csv_file << "," << force_magnitude;
    }
    
    // Vai alla riga successiva
    forces_csv_file << "\n";
}

void CableModelPlugin::write_positions(){
    // solo su Z
    // Ottieni il tempo corrente
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();

    // Scrivi il tempo corrente sul file
    position_csv_file << elapsed;

    // Per ogni massa, ottieni la posizione lungo Z 
    for (int i = 0; i < this->num_particles; ++i) {
        auto position = cable->getPositionWrtWorld(i);
        
        // Calcolo del modulo della forza
        double position_x = position.X();
        double position_z = position.Z();

        // Scrivi il modulo della forza sul CSV
        position_csv_file << "," << position_x << "," << position_z;
    }

    // Vai alla riga successiva
    position_csv_file << "\n";
}



ignition::math::Quaterniond
CableModelPlugin::computeMass0FinalRotation(
  const ignition::math::Quaterniond &robot_rot,
  const ignition::math::Quaterniond &curr_mass_rot)
{
  // primo ciclo: memorizzo e restituisco la rotazione corrente
  if (!this->prev1_initialized_) {
    this->prev_robot1_rot_   = robot_rot;
    this->prev1_initialized_ = true;
    return curr_mass_rot;
  }

  // calcolo delta
  auto delta = robot_rot * this->prev_robot1_rot_.Inverse();
  // aggiorno prev
  this->prev_robot1_rot_ = robot_rot;
  // applico delta alla rotazione corrente della massa
  return delta * curr_mass_rot;
}

// === massa N ===
ignition::math::Quaterniond
CableModelPlugin::computeMassNFinalRotation(
  const ignition::math::Quaterniond &robot_rot,
  const ignition::math::Quaterniond &curr_mass_rot)
{
  if (!this->prev2_initialized_) {
    this->prev_robot2_rot_   = robot_rot;
    this->prev2_initialized_ = true;
    return curr_mass_rot;
  }

  auto delta = robot_rot * this->prev_robot2_rot_.Inverse();
  this->prev_robot2_rot_ = robot_rot;
  return delta * curr_mass_rot;
}