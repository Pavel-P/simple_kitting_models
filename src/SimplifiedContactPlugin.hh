#ifndef _SIMPLE_CONTACT_PLUGIN_HH_
#define _SIMPLE_CONTACT_PLUGIN_HH_

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <boost/algorithm/string.hpp>


namespace gazebo
{
  class SimplifiedContact: public ModelPlugin
  {
    class VirtualJoint
    {
      public: physics::JointPtr joint;
      public: SimplifiedContact* parent;
      public: physics::Collision* collision1;
      public: physics::Collision* collision2;
      public: ignition::math::Vector3d contactForce;

      public: static bool CheckValidity(VirtualJoint* v);

      public: VirtualJoint(SimplifiedContact* p,
                           physics::JointPtr j,
                           physics::Collision* c1,
                           physics::Collision* c2,
                           ignition::math::Vector3d f);
      public: ~VirtualJoint() {};
    };

    private: std::vector<VirtualJoint*> virtualJoints;

    private: std::vector<std::string> collisionNames;
    private: double attachThreshold;
    private: double detachThreshold;
    private: double rate;

    private: physics::ContactManager* contactManager;
    private: physics::ModelPtr model;
    private: unsigned int jointId;


    private: event::ConnectionPtr updateConnection;

    public: SimplifiedContact() {};
    public: ~SimplifiedContact() {};

    public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    public: virtual void OnUpdate();
  };
}
#endif
