#include "WorldCablePlugin.hpp"


using namespace std;
using namespace gazebo;
using namespace sdf_sphere;
using namespace sdf_builder;





CableSpawner::CableSpawner() : WorldPlugin() {
    std::cout << "Starting cable world plugin..." << std::endl;
    // t0 = std::chrono::steady_clock::now();
}
CableSpawner::~CableSpawner(){
    ros::shutdown();
    std::cout << "cable world plugin stopped." << std::endl;
}

 
void CableSpawner::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf_world)
{ 

    
    checkROSInitializzation(); // Make sure the ROS node for Gazebo has already been initialized

    readCableParameters(_sdf_world); // Read the parameters from the SDF file.world that you launch with gazebo
    this->cableInfoMsg();
    
    const float discrete_mass = this->mass / num_particles;
    double pos[3] = {pos_x, pos_y, pos_z};
    
    string sdf_result;
    
    if(cable_type == "cylinder") {
        cylinder_sdf.setModelName("cable");
        cylinder_sdf.setModelPose(this->pos_x, this->pos_y, this->pos_z, this->rot_x, this->rot_y, this->rot_z);
        
        for(int i = 0; i <= this->num_particle_links; i++){ 
            // Calculate position for this mass
            float mass_x = this->pos_x + (i * resolution);
            float mass_y = this->pos_y;
            float mass_z = this->pos_z;
            
            // Calculate orientation for cylinder representing the link
            float cylinder_rot_x = this->rot_x;
            float cylinder_rot_y = this->rot_y; 
            float cylinder_rot_z = this->rot_z;
            
            // For links (cylinders between masses), orient along cable direction
            if(i < this->num_particle_links) {
                // This cylinder represents the link from mass i to mass i+1
                float current_mass_x = this->pos_x + (i * resolution);
                float next_mass_x = this->pos_x + ((i + 1) * resolution);
                
                // Calculate link direction vector
                float link_dir_x = next_mass_x - current_mass_x;
                float link_dir_y = 0.0; // Initially straight
                float link_dir_z = 0.0;
                
                // Normalize direction
                float link_length = sqrt(link_dir_x*link_dir_x + link_dir_y*link_dir_y + link_dir_z*link_dir_z);
                if(link_length > 1e-6) {
                    link_dir_x /= link_length;
                    link_dir_y /= link_length;
                    link_dir_z /= link_length;
                    
                    // Calculate rotation to align cylinder (default Z-axis) with link direction
                    // For a straight cable along X-axis, we need rotation around Y-axis
                    cylinder_rot_y = atan2(link_dir_x, link_dir_z);
                    cylinder_rot_x = -atan2(link_dir_y, sqrt(link_dir_x*link_dir_x + link_dir_z*link_dir_z));
                }
                
                // Position cylinder at the center of the link
                mass_x = current_mass_x + (link_dir_x * resolution * 0.5);
                mass_y = mass_y + (link_dir_y * resolution * 0.5);
                mass_z = mass_z + (link_dir_z * resolution * 0.5);
            }
            
            cylinder_sdf.addLink(this->prefix_mass_names + std::to_string(i), discrete_mass, this->width/2, resolution, 
                               vector<float>{mass_x, mass_y, mass_z, cylinder_rot_x, cylinder_rot_y, cylinder_rot_z});
            cylinder_sdf.setSelfCollide(true);
            cylinder_sdf.setMu1(0.1);
            cylinder_sdf.setMu2(0.2);
            cylinder_sdf.setTortionalFriction(0.1);
            cylinder_sdf.setGravity(gravity);
        }
        cylinder_sdf.addPLugin("WorldCablePlugin", "libmodel_push.so");
        sdf_result = cylinder_sdf.getSDF();
    } 
    else {
        sphere_sdf.setModelName("cable");
        sphere_sdf.setModelPose(this->pos_x, this->pos_y, this->pos_z, this->rot_x, this->rot_y, this->rot_z);
        
        for(int i = 0; i <= this->num_particle_links; i++){ 
            sphere_sdf.addLink(this->prefix_mass_names + std::to_string(i), discrete_mass, this->width/2, vector<float>{this->pos_x + (i*resolution), this->pos_y, this->pos_z, this->rot_x, this->rot_y, this->rot_z});
            sphere_sdf.setSelfCollide(true);
            sphere_sdf.setMu1(0.1);
            sphere_sdf.setMu2(0.2);
            sphere_sdf.setTortionalFriction(0.1);
            sphere_sdf.setGravity(gravity);
        }
        sphere_sdf.addPLugin("WorldCablePlugin", "libmodel_push.so");
        sdf_result = sphere_sdf.getSDF();
    }

    sdf::SDF cableSDF;
    cableSDF.SetFromString(sdf_result);
    _parent->InsertModelSDF(cableSDF);


}


void CableSpawner::pushParamToParamServer(){
   
  this-> checkROSInitializzation();
    
  ros_nh.setParam("/cable/length", this->length);
  ros_nh.setParam("/cable/width", this->width);
  ros_nh.setParam("/cable/resolution", this->resolution);
  ros_nh.setParam("/cable/position", std::vector<double>({this->pos_x, this->pos_y, this->pos_z}));
  ros_nh.setParam("/cable/rotation", std::vector<double>({this->rot_x, this->rot_y, this->rot_z}));
  ros_nh.setParam("/cable/mass", this->mass);
  ros_nh.setParam("/cable/gravity", this->gravity);
  ros_nh.setParam("/cable/damping", this->damping);
  ros_nh.setParam("/cable/torsion_damper", this->torsion_damper);
  ros_nh.setParam("/cable/young_modulus", this->young_modulus);
  ros_nh.setParam("/cable/poisson_ratio", this->poisson_ratio);
  ros_nh.setParam("/cable/num_particles", this->num_particles);
  ros_nh.setParam("/cable/nm_of_springs", this->num_particle_links);
  ros_nh.setParam("/cable/prefix_mass_names", this->prefix_mass_names);
}

void CableSpawner::readCableParameters(sdf::ElementPtr _sdf)
{
  // TROVARE UN ALGORITMO CHE ALZI ERRORE NEL CASO IN CUI UN PARAMETRO NON SIA STATO SETTATO DENTRO AL FILE .WORLD
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
  
  this->resolution = (this->length / static_cast<float>(this->num_particle_links)); // dal paper l_i = l_tot/(num_masses -1)
  

  
  this->pushParamToParamServer();

}


void CableSpawner::cableInfoMsg()
{

    ROS_INFO_STREAM("width = "                  << this->width);
    ROS_INFO_STREAM("length = "                 << this->length);
    ROS_INFO_STREAM("pos_x = "                  << this->pos_x);
    ROS_INFO_STREAM("pos_y = "                  << this->pos_y);
    ROS_INFO_STREAM("pos_z = "                  << this->pos_z);
    ROS_INFO_STREAM("rot_x = "                  << this->rot_x);
    ROS_INFO_STREAM("rot_y = "                  << this->rot_y);
    ROS_INFO_STREAM("rot_z = "                  << this->rot_z);
    ROS_INFO_STREAM("mass = "                   << this->mass);
    ROS_INFO_STREAM("damping = "                << this->damping);
    ROS_INFO_STREAM("torsion_damper = "         << this->torsion_damper);
    ROS_INFO_STREAM("young_modulus = "          << this->young_modulus);
    ROS_INFO_STREAM("poisson_ratio = "          << this->poisson_ratio);
    ROS_INFO_STREAM("gravity = "                << this->gravity);
    ROS_INFO_STREAM("resolution = "             << this->resolution);
    ROS_INFO_STREAM("num_particles = "          << this->num_particles);
    ROS_INFO_STREAM("prefix_mass_names = "      << this->prefix_mass_names);
    ROS_INFO_STREAM("cable_type = "             << this->cable_type);
    
    
}


void CableSpawner::checkROSInitializzation(){
  if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }
}




