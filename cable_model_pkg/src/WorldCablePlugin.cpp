#include "WorldCablePlugin.hpp"


using namespace std;
using namespace gazebo;
using namespace sdf_sphere;
using namespace sdf_builder;





CableSpawner::CableSpawner() : WorldPlugin(), SphereSdf() {
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
    

    SphereSdf::setModelName("cable");
    SphereSdf::setModelPose(this->pos_x, this->pos_y, this->pos_z, this->rot_x, this->rot_y, this->rot_z);

    
    const float discrete_mass = this->mass / num_particles;
    // cout<<discrete_mass<<endl;
    
    double pos[3] = {pos_x, pos_y, pos_z};
    
    for(int i = 0; i <= this->num_particle_links; i++){ // Create the mass points of the cable 
        SphereSdf::addLink(this->prefix_mass_names + std::to_string(i), discrete_mass, this->width/2, vector<float>{this->pos_x + (i*resolution), this->pos_y, this->pos_z, this->rot_x, this->rot_y, this->rot_z});
        setSelfCollide(true);
        setMu1(0.1);
        setMu2(0.2);
        setTortionalFriction(0.1);
        setGravity(gravity);

    }

    SphereSdf::addPLugin("WorldCablePlugin", "libmodel_push.so");
    // cout << getSDF() << endl;
    // //Demonstrate using a custom model name.
    sdf::SDF sphereSDF;
    sphereSDF.SetFromString(getSDF());
    // sdf::ElementPtr model = sphereSDF.Root()->GetElement("cable");
    // model->GetAttribute("name")->SetFromString("unique_sphere");
    _parent->InsertModelSDF(sphereSDF);


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
    ROS_INFO_STREAM("young_modulus = "          << this->young_modulus);
    ROS_INFO_STREAM("poisson_ratio = "          << this->poisson_ratio);
    ROS_INFO_STREAM("gravity = "                << this->gravity);
    ROS_INFO_STREAM("resolution = "             << this->resolution);
    ROS_INFO_STREAM("num_particles = "          << this->num_particles);
    ROS_INFO_STREAM("prefix_mass_names = "      << this->prefix_mass_names);
    
    
}


void CableSpawner::checkROSInitializzation(){
  if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }
}




