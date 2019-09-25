#include "SimplifiedContactPlugin.hh"
#include <algorithm>
#include <cassert>
#include <chrono>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(SimplifiedContact)


void SimplifiedContact::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  gzdbg << "Loading plugin..." << std::endl;

  std::string rawCollisionNames = "";

  this->attachThreshold = 0.1;
  if (_sdf->HasElement("attach_threshold"))
    this->attachThreshold = _sdf->Get<double>("attach_threshold");

  this->detachThreshold = 0.1;
  if (_sdf->HasElement("detach_threshold"))
    this->detachThreshold = _sdf->Get<double>("detach_threshold");

  this->rate = 10.0;
  if (_sdf->HasElement("rate"))
    this->rate = _sdf->Get<double>("rate");

  if (_sdf->HasElement("collision_names"))
    rawCollisionNames = _sdf->Get<std::string>("collision_names");
  else
    gzdbg << "No collision names given. Plugin not active." <<std::endl;

  boost::split(this->collisionNames, rawCollisionNames, boost::is_any_of(" "));

  std::transform(this->collisionNames.begin(),
                 this->collisionNames.end(),
                 this->collisionNames.begin(),
                [_parent](auto name){return _parent->GetName() + "::" + name;});


  //for (auto name : this->collisionNames)
  //  gzdbg << "Found collision name: " << name << std::endl;

  this->model = _parent;
  this->jointId = 0;

  this->contactManager = _parent->GetWorld()->Physics()->GetContactManager();
  this->contactManager->SetNeverDropContacts(true);
  

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&SimplifiedContact::OnUpdate, this));

  gzdbg << "Loading Complete!"<< std::endl;
}

int count = 0;

void SimplifiedContact::OnUpdate()
{
  // **** Check for Deletion ****
  auto start = std::chrono::high_resolution_clock::now();
  
  //auto it = std::remove_if(this->virtualJoints.begin(),
  //                         this->virtualJoints.end(),
  //                         SimplifiedContact::VirtualJoint::CheckValidity);
  //
  //this->virtualJoints.erase(it, this->virtualJoints.end());

  if (!this->virtualJoints.empty())
  {
    auto savedForce = ignition::math::Vector3d();
    auto currentForce = ignition::math::Vector3d();

    for (auto v : this->virtualJoints)
    {
      auto parentPose = v->joint->GetParent()->WorldPose();
      auto childPose = v->joint->GetChild()->WorldPose();

      savedForce += v->contactForce;
      currentForce += -1 * v->joint->GetForceTorque(0).body2Force;
      
      //gzdbg << "This virtual joint's saved force is: " << v->contactForce << std::endl;
      //gzdbg << "This virtual joint's current force1 is: " << v->joint->GetForceTorque(0).body1Force << std::endl;
      //gzdbg << "This virtual joint's current force2 is: " << v->joint->GetForceTorque(0).body2Force << std::endl;
      //gzdbg << "This virtual joint's child world pose is: " << ignition::math::Matrix3d(v->joint->GetChild()->WorldPose().Rot()) << std::endl;
      //gzdbg << "This virtual joint's parent world pose is: " << ignition::math::Matrix3d(v->joint->GetParent()->WorldPose().Rot()) << std::endl;

    }

    double modelMass = 0.;
    for (auto link : this->model->GetLinks())
    {
      modelMass += link->GetInertial()->Mass();
    }
    auto gravity = this->model->WorldPose().Inverse().Rot() * this->model->GetWorld()->Gravity() * modelMass;

    //gzdbg << "Saved Force is: " << savedForce << std::endl;
    //gzdbg << "Current Force is: " << currentForce << std::endl;

    //currentForce -= gravity;
    //gzdbg << "Current Force (Gravity adjusted) is: " << currentForce << std::endl;

    auto error = (savedForce - currentForce).Length();

    //gzdbg << "Error is: " << error << std::endl; 


    if (error > this->detachThreshold)
    {
      //gzdbg << "Error is: " << error << ", Deleting all virtual joints." << std::endl;
      for (auto v : this->virtualJoints)
      {
        v->collision1->SetCollideBits(0xffff);
        v->collision2->SetCollideBits(0xffff);
        v->parent->model->RemoveJoint(v->joint->GetName());
      }
      this->virtualJoints.clear();
    }
  }

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  //gzdbg << "Joint Deletion step took this many microseconds: " << duration.count() << std::endl;

  if (count++ % int(1000. / this->rate)) return;

  // **** Check for Creation ****
  start = std::chrono::high_resolution_clock::now();

  auto contacts = this->contactManager->GetContacts();

  //gzdbg << "Found this many contacts: " << this->contactManager->GetContactCount() << std::endl;

  //for (auto vj : this->virtualJoints)
  //  gzdbg << "Found virtual joint: " << vj->joint->GetName() << std::endl;

  for (auto contact : contacts)
  {
    std::string collision1 = contact->collision1->GetScopedName();
    std::string collision2 = contact->collision2->GetScopedName();

    physics::Collision* internalCollision = NULL;
    physics::Collision* externalCollision = NULL;
    bool foundCollision = false;

    if (std::find(this->collisionNames.begin(), this->collisionNames.end(), collision1) != this->collisionNames.end())
    {
       internalCollision = contact->collision1;
       externalCollision = contact->collision2;
       foundCollision = true;
    }
    else if (std::find(this->collisionNames.begin(), this->collisionNames.end(), collision2) != this->collisionNames.end())
    {
       internalCollision = contact->collision2;
       externalCollision = contact->collision1;
       foundCollision = true;
    }
    if (foundCollision)
    {
      assert(internalCollision->GetModel() == this->model);
      assert(externalCollision->GetModel() != this->model);

      /*gzdbg << "Internal Collision Name: " << internalCollision->GetName() << std::endl;
      gzdbg << "Internal World Linear Accel: " << internalCollision->WorldLinearAccel() << std::endl;
      gzdbg << "Internal World Angular Accel: " << internalCollision->WorldAngularAccel() << std::endl;
      gzdbg << "Internal World Linear Vel: " << internalCollision->WorldLinearVel() << std::endl;
      gzdbg << "Internal World Angular Vel: " << internalCollision->WorldAngularVel() << std::endl;
      gzdbg << "External Collision Name: " << externalCollision->GetName() << std::endl;
      gzdbg << "External World Linear Accel: " << externalCollision->WorldLinearAccel() << std::endl;
      gzdbg << "External World Angular Accel: " << externalCollision->WorldAngularAccel() << std::endl;
      gzdbg << "External World Linear Vel: " << externalCollision->WorldLinearVel() << std::endl;
      gzdbg << "External World Angular Vel: " << externalCollision->WorldAngularVel() << std::endl;*/

      auto relativeLinearVel    = internalCollision->WorldLinearVel() - externalCollision->WorldLinearVel();
      auto relativeLinearAccel  = internalCollision->WorldLinearAccel() - externalCollision->WorldLinearAccel();
      auto relativeAngularVel   = internalCollision->WorldAngularVel() - externalCollision->WorldAngularVel();
      auto relativeAngularAccel = internalCollision->WorldAngularAccel() - externalCollision->WorldAngularAccel();

      /*gzdbg << "Relative Angular Accel: " << relativeAngularAccel << std::endl;
      gzdbg << "Relative Angular Accel: " << relativeAngularVel << std::endl;
      gzdbg << "Relative Linear Vel: " << relativeLinearVel << std::endl;
      gzdbg << "Relative Linear Accel: " << relativeLinearAccel << std::endl;*/

      if (relativeLinearVel.Length() < this->attachThreshold && !externalCollision->GetModel()->IsStatic())
      {
        // gzdbg << "Relative Vel is: " << relativeLinearVel << ", creating joint." << std::endl;

        auto contactForce = ignition::math::Vector3d();
        
        if (internalCollision == contact->collision1)
        {
          for (int i = 0; i < sizeof(contact->wrench)/sizeof(physics::JointWrench); i++)
            contactForce += contact->wrench[i].body1Force;
        }
        else
        {
          for (int i = 0; i < sizeof(contact->wrench)/sizeof(physics::JointWrench); i++)
            contactForce += contact->wrench[i].body2Force;
        }
        auto jointName = this->model->GetName() + "_virtual_joint_" + std::to_string(this->jointId++);
        auto joint = this->model->CreateJoint(jointName, "fixed",
                                              externalCollision->GetLink(),
                                              internalCollision->GetLink());
        joint->SetProvideFeedback(true);
        auto offset = internalCollision->WorldPose() - externalCollision->WorldPose();
        joint->Load(externalCollision->GetLink(), internalCollision->GetLink(), offset);
        joint->Init();

        this->virtualJoints.push_back(new VirtualJoint(this, joint, internalCollision, externalCollision, contactForce));

        internalCollision->SetCollideBits(0x00ff);
        externalCollision->SetCollideBits(0xff00);
      }
    }
  }
  this->contactManager->Clear();

  stop = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  //gzdbg << "Joint creation step took this many microseconds: " << duration.count() << std::endl;
}


SimplifiedContact::VirtualJoint::VirtualJoint(SimplifiedContact* p,
                                              physics::JointPtr j,
                                              physics::Collision* c1,
                                              physics::Collision* c2,
                                              ignition::math::Vector3d f)
  : parent(p), joint(j), collision1(c1), collision2(c2), contactForce(f)
{
  //gzdbg << "Got joint: " << this->joint->GetScopedName() << std::endl;
}

bool SimplifiedContact::VirtualJoint::CheckValidity(VirtualJoint* v)
{
  //Joint Deletion Callback here
  auto savedForce = v->contactForce;
  auto currentForce = -v->joint->GetForceTorque(0).body2Force;
  auto error = (savedForce - currentForce).Length();

  //gzdbg << "Saved Force is: " << savedForce << std::endl;
  //gzdbg << "Current Force is: " << currentForce << std::endl;
  //gzdbg << "Error is: " << error << std::endl; 

  if (error > v->parent->detachThreshold)
  {
    //gzdbg << "Error is: " << error << ", Deleting virtual joint: " << v->joint->GetName() << std::endl;

    v->collision1->SetCollideBits(0xffff);
    v->collision2->SetCollideBits(0xffff);
    v->parent->model->RemoveJoint(v->joint->GetName());

    return true;
  }

  return false;
}
