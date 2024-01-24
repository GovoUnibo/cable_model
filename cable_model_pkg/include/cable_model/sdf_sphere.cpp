#include <sdf_sphere.hpp>

using namespace sdf_builder;
using namespace sdf_sphere;
using namespace std;

SphereSdf::SphereSdf() : SdfBuilder()
{ 
    SdfBuilder::setModelType(sphere);
}
SphereSdf::~SphereSdf(){}

void SphereSdf::setModelName(std::string model_name)    { SdfBuilder::setModelName(model_name); }

void SphereSdf::setLinkName(std::string link_name){
    num_of_link += 1; 
    SdfBuilder::setLinkName(link_name); 
    this->mass.push_back((1));
    this->radius.push_back(1);
    SphereSdf::setFriction(1, 1, vector<float>{0,0,0}, 0, 0);
    }

void SphereSdf::setModelPose(float x=0.0, float y=0.0, float z=0.0, float roll=0.0, float pitch=0.0, float yaw=0.0) {SdfBuilder::setModelPose(x,y,z,roll,pitch,yaw);}

void SphereSdf::setLinkPose(float x=0.0, float y=0.0, float z=0.0, float roll=0.0, float pitch=0.0, float yaw=0.0) {SdfBuilder::setLinkPose(x,y,z,roll,pitch,yaw);}

void SphereSdf::setRadius(float radius)
{ 
    this->radius.back() = radius;
    SdfBuilder::setLinkDimension(to_string(radius));
}

void SphereSdf::setMass(float mass)
{
    this->mass.back() = mass;
    SdfBuilder::setMass(mass);
}

void SphereSdf::setSelfCollide(bool self_collide){SdfBuilder::setSelfCollide(self_collide);}

void SphereSdf::setGravity(bool gravity){SdfBuilder::setGravity(gravity);}

void SphereSdf::setMu1(float mu1){SdfBuilder::setMu1(mu1);this->mu1.back()= mu1;}
void SphereSdf::setMu2(float mu2){SdfBuilder::setMu2(mu2);}
void SphereSdf::setFdir1(vector<float> fdir1= vector<float>{0,0,0}){SdfBuilder::setFdir1(fdir1);}
void SphereSdf::setSlip1(float slip1){SdfBuilder::setSlip1(slip1);}
void SphereSdf::setSlip2(float slip2){SdfBuilder::setSlip2(slip2);}
void SphereSdf::setFriction(float mu1=1, float mu2=1, std::vector<float> fdir1 = vector<float>{0,0,0}, float slip1=0, float slip2=0){
    this->mu1.push_back(mu1);
    this->torsional_friction.push_back(0);
    SdfBuilder::setMu1(mu1);
    SdfBuilder::setMu2(mu2);
    SdfBuilder::setFdir1(fdir1);
    SdfBuilder::setSlip1(slip1);
    SdfBuilder::setSlip2(slip2);
    SdfBuilder::setTortionalFriction(-1.0);
}

void SphereSdf::setTortionalFriction(float contact_depth){
    float N = this->mass.back()*9.81;
    float T = (3*M_PI/16)*(sqrt(this->radius.back()*contact_depth)*this->mu1.back()*N);
    this->torsional_friction.back() = T;
    SdfBuilder::setTortionalFriction(T);

}
//void SphereSdf::addLink(string link_name, float mass, float radius, vector<float> pose)
void SphereSdf::addLink(string link_name="", float mass=1, float radius=1, vector<float> pose= vector<float>{0,0,0,0,0,0}){

    SphereSdf::setLinkName(link_name);
    SphereSdf::setLinkPose(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    SphereSdf::setMass(mass);
    
    SphereSdf::setRadius(radius);
    

}



void SphereSdf::addJoint(std::string parent, std::string child, TypeOfJoint type, vector<int> axis= vector<int>{0,0,0}){   

    string joint_name = parent + "_to_" + child;
    if( (int)num_of_link>0)
        SdfBuilder::setJoint(joint_name, parent, child, type, axis);
}

void SphereSdf::addJoint(TypeOfJoint type, vector<int> axis= vector<int>{0,0,0}){
    if( (int)num_of_link>0){
        string parent = SdfBuilder::getLinkName(num_of_link-1);
        string child = SdfBuilder::getLinkName(num_of_link);
        cout << parent << "," << child << endl;
         string joint_name = parent + "_to_" + child;
        SdfBuilder::setJoint(joint_name, parent, child, type, axis);
    }
}

void SphereSdf::addPLugin(std::string plugin_name, std::string plugin_filename){
    SdfBuilder::setPlugin(plugin_name, plugin_filename);
}

void SphereSdf::computeInertiaMatrix(float mass, float radius){
    float i_xx, i_xy, i_xz, i_yy, i_yz, i_zz;

    i_xx = (2.0/5.0)*(mass*pow(radius,2));
    i_xy =  0.0;
    i_xz =  0.0;
    i_yy = (2.0/5.0)*(mass*pow(radius,2)); 
    i_yz =  0.0;
    i_zz = (2.0/5.0)*(mass*pow(radius,2)); 
    SdfBuilder::setInertiaMomentParam(i_xx, i_xy, i_xz, i_yy, i_yz, i_zz);
}



string SphereSdf::getSDF(){
    
    string open_sdf = SdfBuilder::getXmlVersion() + SdfBuilder::getOpenSdf();
    string model = SdfBuilder::getOpenModel() + SdfBuilder::getModelPose();
    string plugin_list = "";

    string link_list = "";
    for(int i=0; i<=num_of_link; i++)
    {   
        this->computeInertiaMatrix(mass[i], radius[i]);
        link_list  +=   SdfBuilder::getOpenLink(i)      +
                        SdfBuilder::getGravity(i)       +
                        SdfBuilder::getSelfCollision(i) +
                        SdfBuilder::getInertial(i)      +
                        SdfBuilder::getLinkPose(i)      +
                        SdfBuilder::getCollision(i)     +
                        SdfBuilder::getVisual(i)        +
                        SdfBuilder::getCloseLink()
                        ;

    }

    string joint_list = "";



    
    return  open_sdf +
            model +
            link_list +
            SdfBuilder::getJoints() +
            SdfBuilder::getPlugin() +
            SdfBuilder::getCloseModel() +
            SdfBuilder::getCloseSdf();

}
