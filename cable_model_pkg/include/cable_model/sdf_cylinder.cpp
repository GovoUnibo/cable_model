#include "sdf_cylinder.hpp"

using namespace sdf_cylinder;
using namespace std;

CylinderSdf::CylinderSdf(/* args */)
{
    this->setModelType(sdf_builder::cylinder);
}

CylinderSdf::~CylinderSdf()
{
}

void CylinderSdf::addLink(string link_name, float mass, float radius, float length, vector<float> pose)
{
    this->setLinkName(link_name);
    this->setMass(mass);
    this->setLinkPose(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    
    // Set cylinder dimensions: "<radius>value</radius><length>value</length>"
    string cylinder_dimension = "<radius>" + to_string(radius) + "</radius>\n<length>" + to_string(length) + "</length>";
    this->setLinkDimension(cylinder_dimension);
    
    // Calculate inertia for cylinder
    float i_xx = (1.0/12.0) * mass * (3*radius*radius + length*length);
    float i_yy = i_xx;
    float i_zz = (1.0/2.0) * mass * radius * radius;
    
    this->setInertiaMomentParam(i_xx, 0, 0, i_yy, 0, i_zz);
}

void CylinderSdf::setSelfCollide(bool self_collide)
{
    this->sdf_builder::SdfBuilder::setSelfCollide(self_collide);
}

void CylinderSdf::setMu1(float mu1)
{
    this->sdf_builder::SdfBuilder::setMu1(mu1);
}

void CylinderSdf::setMu2(float mu2)
{
    this->sdf_builder::SdfBuilder::setMu2(mu2);
}

void CylinderSdf::setFdir1(vector<float> fdir1)
{
    this->sdf_builder::SdfBuilder::setFdir1(fdir1);
}

void CylinderSdf::setSlip1(float slip1)
{
    this->sdf_builder::SdfBuilder::setSlip1(slip1);
}

void CylinderSdf::setSlip2(float slip2)
{
    this->sdf_builder::SdfBuilder::setSlip2(slip2);
}

void CylinderSdf::setTortionalFriction(float t_f)
{
    this->sdf_builder::SdfBuilder::setTortionalFriction(t_f);
}

void CylinderSdf::setGravity(bool gravity)
{
    this->sdf_builder::SdfBuilder::setGravity(gravity);
}

void CylinderSdf::addPLugin(string plugin_name, string plugin_filename)
{
    this->sdf_builder::SdfBuilder::setPlugin(plugin_name, plugin_filename);
}

string CylinderSdf::getSDF()
{
    string sdf_content = this->getXmlVersion() + 
                        this->getOpenSdf() + 
                        this->getOpenModel() + 
                        this->getModelPose();

    // Add all links
    for(int i = 0; i < this->getNumOfLinks(); i++)
    {
        sdf_content += this->getOpenLink(i) + 
                      this->getLinkPose(i) + 
                      this->getInertial(i) + 
                      this->getCollision(i) + 
                      this->getVisual(i) + 
                      this->getSelfCollision(i) + 
                      this->getGravity(i) + 
                      this->getCloseLink();
    }

    // Add joints if any
    if(this->modelHasJoint())
    {
        sdf_content += this->getJoints();
    }

    // Add plugins if any
    sdf_content += this->getPlugin();

    sdf_content += this->getCloseModel() + this->getCloseSdf();
    
    return sdf_content;
}
