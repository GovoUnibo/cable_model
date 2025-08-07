#include "CablePlugin.hpp"
#include <geometry_msgs/Pose.h>

using namespace gazebo;
using namespace std;
using namespace sdf_sphere;
using namespace sdf_cylinder;
using namespace cable_utils;

void OptimizedCablePlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf_world){
    // Store world pointer
    this->world = _parent;


    
    readCableParameters(_sdf_world); // Read the parameters from the SDF file.world that you launch with gazebo
    this->cableInfoMsg();
    
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, this->model_name + "optimized_cable_plugin",
            ros::init_options::NoSigintHandler);
    }

    checkROSInitializzation(); // Make sure the ROS node for Gazebo has already been initialized
    
    

    const float discrete_mass = this->mass / num_particles;
    double pos[3] = {pos_x, pos_y, pos_z};
    
    string sdf_result;
    
    // Create namespaced model name
   

    sphere_sdf.setModelName(this->model_name);
    sphere_sdf.setModelPose(this->pos_x, this->pos_y, this->pos_z, this->rot_x, this->rot_y, this->rot_z);
        
    for(int i = 0; i <= this->num_particle_links; i++){ 
        sphere_sdf.addLink(this->prefix_mass_names + std::to_string(i), discrete_mass, this->width/2, 
                            vector<float>{this->pos_x + (i*resolution), this->pos_y, this->pos_z, this->rot_x, this->rot_y, this->rot_z});
        sphere_sdf.setSelfCollide(true);
        sphere_sdf.setMu1(0.1);
        sphere_sdf.setMu2(0.2);
        sphere_sdf.setTortionalFriction(0.1);
        sphere_sdf.setGravity(gravity);
    }
    sdf_result = sphere_sdf.getSDF();

    sdf::SDF cableSDF;
    cableSDF.SetFromString(sdf_result);
    _parent->InsertModelSDF(cableSDF);
    
    std::cout << ORANGE << "Cable model created successfully with " << this->num_particles << " particles!" << RESET << std::endl;
    
    // Setup cable control first (subscribers and services)
    setupCableControl();
    
    // Setup cable model object after a short delay to ensure model is loaded
    // We'll do this in the first OnUpdate call
    cable_model_found = false;

    auto phys = this->world->Physics();
    this->baseDt     = phys->GetMaxStepSize(); 
    this->forceThresh = 200.0; // Newton, taralo tu
    this->minScale    = 0.1;   // mai sotto il 10% del baseDt
    this->maxScale    = 1.0;   // mai sopra il 100% del baseDt (no aumento)
    
    // Inizializza variabili per l'ottimizzazione dello step size
    this->stepOptimizationCounter = 0;
    this->stepOptimizationInterval = 10; // Ottimizza ogni 10 frame per ridurre overhead
    this->lastOptimizedScale = 1.0;
    
}

void OptimizedCablePlugin::setupCableControl(){
    // Create namespaced topic names
    std::string mass0_topic =  this->model_name + "/mass0/pose";
    std::string massN_topic =  this->model_name + "/massN/pose";
    std::string service_name = this->model_name + "/set_cable_grasp";
    
    // Setup ROS subscribers for robot poses
    this->sub_mass0_coords = ros_nh.subscribe(mass0_topic, 1, &OptimizedCablePlugin::robot1PoseCallback, this);
    this->sub_massN_coords = ros_nh.subscribe(massN_topic, 1, &OptimizedCablePlugin::robot2PoseCallback, this);
    
    // Setup grasp service
    this->graspServer = ros_nh.advertiseService(service_name, &OptimizedCablePlugin::callbackGraspServer, this);

    // Listen to the update event for cable control
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&OptimizedCablePlugin::OnUpdate, this));


}

void OptimizedCablePlugin::setupCableModel(){
    // Create namespaced model name
    
    // Find the cable model that was just created
    cable_model = world->ModelByName(this->model_name);
    
    if (!cable_model) {
        ROS_ERROR("Cable model '%s' not found! Make sure it was created properly.", model_name.c_str());
        return;
    }
            
    // Initialize cable object for control
    cable = std::make_shared<cable_utils::Cable>(cable_model, this->pos_x, this->pos_y, this->pos_z, 
                                                this->mass, this->width, this->length, this->num_particles, 
                                                this->prefix_mass_names);

    cable->useGravity(this->gravity);
    cable->setDamperCoef(this->damping);
    cable->setYoungModulus(this->young_modulus);
    cable->setPoissonRatio(this->poisson_ratio);
    cable->setTorsionDamperCoef(this->torsion_damper);
    
    std::cout << ORANGE << "Cable model '" << model_name << "' found and cable object initialized!" << RESET << std::endl;
}

// Callback per coordinate complete massa 0 (posizione + rotazione)
void OptimizedCablePlugin::robot1PoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    this->robot1_pos = {msg->position.x, msg->position.y, msg->position.z};
    this->robot1_rot = {msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z};
}

// Callback per coordinate complete massa N (posizione + rotazione)
void OptimizedCablePlugin::robot2PoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    this->robot2_pos = {msg->position.x, msg->position.y, msg->position.z};
    this->robot2_rot = {msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z};
}

bool OptimizedCablePlugin::callbackGraspServer(cable_model_pkg::GraspMsg::Request &req, cable_model_pkg::GraspMsg::Response &res){
    this->isMass0Gripped = req.grasp_mass_0;
    this->isMassNGripped = req.grasp_mass_N;
    std::cout << ORANGE << "Grasp state updated - Mass0: " << this->isMass0Gripped << ", MassN: " << this->isMassNGripped << RESET << std::endl;
    return true;
}
                        
void OptimizedCablePlugin::OnUpdate(){
    // Try to find and setup cable model if not done yet
    if (!cable_model_found) {
        setupCableModel();
        if (cable_model) {
            cable_model_found = true;
        } else {
            return; // Return early if cable model not ready yet
        }
    }

    if (!cable) return; // Safety check



    // MASSA 0
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

    // Ottimizzazione del step size adattivo (non ad ogni frame per performance)
    auto phys = this->cable_model->GetWorld()->Physics();
    double scale = this->lastOptimizedScale; // Usa l'ultimo valore calcolato
    
    // Calcola nuovo scale solo ogni stepOptimizationInterval frame
    if (this->stepOptimizationCounter % this->stepOptimizationInterval == 0) {
        scale = this->cable->getStepScale(this->baseDt);
        scale = std::clamp(scale, this->minScale, this->maxScale);
        this->lastOptimizedScale = scale;
    }
    this->stepOptimizationCounter++;
    
    phys->SetMaxStepSize(this->baseDt * scale);
}

ignition::math::Quaterniond OptimizedCablePlugin::computeMass0FinalRotation(const ignition::math::Quaterniond &robot_rot, const ignition::math::Quaterniond &curr_mass_rot) {
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
ignition::math::Quaterniond OptimizedCablePlugin::computeMassNFinalRotation(const ignition::math::Quaterniond &robot_rot, const ignition::math::Quaterniond &curr_mass_rot) {
  if (!this->prev2_initialized_) {
    this->prev_robot2_rot_   = robot_rot;
    this->prev2_initialized_ = true;
    return curr_mass_rot;
  }

  auto delta = robot_rot * this->prev_robot2_rot_.Inverse();
  this->prev_robot2_rot_ = robot_rot;
  return delta * curr_mass_rot;
}



void OptimizedCablePlugin::readCableParameters(sdf::ElementPtr _sdf)
{
    if(_sdf->HasElement("model_name"))
        model_name = _sdf->Get<std::string>("model_name");

    if(_sdf->HasElement("width"))
        width = _sdf->Get<float>("width");

    if(_sdf->HasElement("length"))
        length = _sdf->Get<float>("length");

    if(_sdf->HasElement("pos_x"))
        pos_x = _sdf->Get<float>("pos_x");

    if(_sdf->HasElement("pos_y"))
        pos_y = _sdf->Get<float>("pos_y");
    
    if(_sdf->HasElement("pos_z"))
        pos_z = _sdf->Get<float>("pos_z");
    
    if(_sdf->HasElement("rot_x"))
        rot_x = _sdf->Get<float>("rot_x");
    
    if(_sdf->HasElement("rot_y"))
        rot_y = _sdf->Get<float>("rot_y");
    
    if(_sdf->HasElement("rot_z"))
        rot_z = _sdf->Get<float>("rot_z");

    if(_sdf->HasElement("mass"))
        mass = _sdf->Get<float>("mass");         

    if(_sdf->HasElement("damping"))
        damping = _sdf->Get<float>("damping");
    
    if(_sdf->HasElement("torsion_damper"))
        torsion_damper = _sdf->Get<float>("torsion_damper");

    if(_sdf->HasElement("young_modulus"))
        young_modulus = _sdf->Get<float>("young_modulus");

    if(_sdf->HasElement("poisson_ratio"))
        poisson_ratio = _sdf->Get<float>("poisson_ratio");

    if(_sdf->HasElement("gravity"))
        gravity = _sdf->Get<bool>("gravity");

    if(_sdf->HasElement("cable_masses")){
        this->num_particles = _sdf->Get<int>("cable_masses");
        num_particle_links = this->num_particles - 1;
    }

    if(_sdf->HasElement("mass_name_prefix"))
        prefix_mass_names = _sdf->Get<string>("mass_name_prefix");
    
    if(_sdf->HasElement("cable_type"))
        cable_type = _sdf->Get<string>("cable_type");
    
    this->resolution = (this->length / static_cast<float>(this->num_particle_links));
    

}

void OptimizedCablePlugin::cableInfoMsg()
{
    ROS_INFO_STREAM(ORANGE << "=== OPTIMIZED CABLE PLUGIN PARAMETERS ===" << RESET);
    ROS_INFO_STREAM(ORANGE << "MODEL NAME = " << this->model_name << RESET);
    ROS_INFO_STREAM(ORANGE << "width = " << this->width << RESET);
    ROS_INFO_STREAM(ORANGE << "length = " << this->length << RESET);
    ROS_INFO_STREAM(ORANGE << "pos_x = " << this->pos_x << RESET);
    ROS_INFO_STREAM(ORANGE << "pos_y = " << this->pos_y << RESET);
    ROS_INFO_STREAM(ORANGE << "pos_z = " << this->pos_z << RESET);
    ROS_INFO_STREAM(ORANGE << "rot_x = " << this->rot_x << RESET);
    ROS_INFO_STREAM(ORANGE << "rot_y = " << this->rot_y << RESET);
    ROS_INFO_STREAM(ORANGE << "rot_z = " << this->rot_z << RESET);
    ROS_INFO_STREAM(ORANGE << "mass = " << this->mass << RESET);
    ROS_INFO_STREAM(ORANGE << "damping = " << this->damping << RESET);
    ROS_INFO_STREAM(ORANGE << "torsion_damper = " << this->torsion_damper << RESET);
    ROS_INFO_STREAM(ORANGE << "young_modulus = " << this->young_modulus << RESET);
    ROS_INFO_STREAM(ORANGE << "poisson_ratio = " << this->poisson_ratio << RESET);
    ROS_INFO_STREAM(ORANGE << "gravity = " << this->gravity << RESET);
    ROS_INFO_STREAM(ORANGE << "resolution = " << this->resolution << RESET);
    ROS_INFO_STREAM(ORANGE << "num_particles = " << this->num_particles << RESET);
    ROS_INFO_STREAM(ORANGE << "prefix_mass_names = " << this->prefix_mass_names << RESET);
    ROS_INFO_STREAM(ORANGE << "cable_type = " << this->cable_type << RESET);
    ROS_INFO_STREAM(ORANGE << "===========================================" << RESET);
}

void OptimizedCablePlugin::checkROSInitializzation(){
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(OptimizedCablePlugin)
