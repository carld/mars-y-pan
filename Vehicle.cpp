//
// Copyright (c) 2008-2016 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#include <Urho3D/Core/Context.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Physics/Constraint.h>
#include <Urho3D/Physics/PhysicsEvents.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>

#include "Vehicle.h"

Vehicle::Vehicle(Context* context) :
    LogicComponent(context),
    steering_(0.0f)
{
    // Only the physics update event is needed: unsubscribe from the rest for optimization
    SetUpdateEventMask(USE_FIXEDUPDATE);
}

void Vehicle::RegisterObject(Context* context)
{
    context->RegisterFactory<Vehicle>();

    URHO3D_ATTRIBUTE("Controls Yaw", float, controls_.yaw_, 0.0f, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Controls Pitch", float, controls_.pitch_, 0.0f, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Steering", float, steering_, 0.0f, AM_DEFAULT);
    // Register wheel node IDs as attributes so that the wheel nodes can be reaquired on deserialization. They need to be tagged
    // as node ID's so that the deserialization code knows to rewrite the IDs in case they are different on load than on save
    URHO3D_ATTRIBUTE("Front Left Node", unsigned, frontLeftID_, 0, AM_DEFAULT | AM_NODEID);
    URHO3D_ATTRIBUTE("Front Right Node", unsigned, frontRightID_, 0, AM_DEFAULT | AM_NODEID);
    URHO3D_ATTRIBUTE("Mid Left Node", unsigned, midLeftID_, 0, AM_DEFAULT | AM_NODEID);
    URHO3D_ATTRIBUTE("Mid Right Node", unsigned, midRightID_, 0, AM_DEFAULT | AM_NODEID);
    URHO3D_ATTRIBUTE("Rear Left Node", unsigned, rearLeftID_, 0, AM_DEFAULT | AM_NODEID);
    URHO3D_ATTRIBUTE("Rear Right Node", unsigned, rearRightID_, 0, AM_DEFAULT | AM_NODEID);
}

void Vehicle::ApplyAttributes()
{
    // This function is called on each Serializable after the whole scene has been loaded. Reacquire wheel nodes from ID's
    // as well as all required physics components
    Scene* scene = GetScene();

    frontLeft_ = scene->GetNode(frontLeftID_);
    frontRight_ = scene->GetNode(frontRightID_);
    midLeft_ = scene->GetNode(midLeftID_);
    midRight_ = scene->GetNode(midRightID_);
    rearLeft_ = scene->GetNode(rearLeftID_);
    rearRight_ = scene->GetNode(rearRightID_);

    hullBody_ = node_->GetComponent<RigidBody>();

    GetWheelComponents();
}

void Vehicle::FixedUpdate(float timeStep)
{
    float newSteering = 0.0f;
    float accelerator = 0.0f;

    // Read controls
    if (controls_.buttons_ & CTRL_LEFT)
        newSteering = -1.0f;
    if (controls_.buttons_ & CTRL_RIGHT)
        newSteering = 1.0f;
    if (controls_.buttons_ & CTRL_FORWARD)
        accelerator = 1.0f;
    if (controls_.buttons_ & CTRL_BACK)
        accelerator = -0.5f;

    // When steering, wake up the wheel rigidbodies so that their orientation is updated
    if (newSteering != 0.0f)
    {
        frontLeftBody_->Activate();
        frontRightBody_->Activate();

        rearLeftBody_->Activate();
        rearRightBody_->Activate();

        steering_ = steering_ * 0.95f + newSteering * 0.05f;
    }
    else
        steering_ = steering_ * 0.8f + newSteering * 0.2f;

    // Set front wheel angles
    Quaternion steeringRot(0, steering_ * MAX_WHEEL_ANGLE, 0);
    frontLeftAxis_->SetOtherAxis(steeringRot * Vector3::LEFT);
    frontRightAxis_->SetOtherAxis(steeringRot * Vector3::RIGHT);

    // Set rear wheel angles
    Quaternion steeringRotInv(0, -steering_ * MAX_WHEEL_ANGLE, 0);
    rearLeftAxis_->SetOtherAxis(  steeringRotInv * Vector3::LEFT);
    rearRightAxis_->SetOtherAxis( steeringRotInv * Vector3::RIGHT);

    Quaternion hullRot = hullBody_->GetRotation();
    if (accelerator != 0.0f)
    {
        // Torques are applied in world space, so need to take the vehicle & wheel rotation into account
        Vector3 torqueVec = Vector3(ENGINE_POWER * accelerator, 0.0f, 0.0f);

        frontLeftBody_->ApplyTorque(hullRot * steeringRot * torqueVec);
        frontRightBody_->ApplyTorque(hullRot * steeringRot * torqueVec);

        midLeftBody_->ApplyTorque(hullRot * torqueVec);
        midRightBody_->ApplyTorque(hullRot * torqueVec);

        rearLeftBody_->ApplyTorque(hullRot * steeringRot * torqueVec);
        rearRightBody_->ApplyTorque(hullRot * steeringRot * torqueVec);
    }

    // Apply downforce proportional to velocity
    Vector3 localVelocity = hullRot.Inverse() * hullBody_->GetLinearVelocity();
    hullBody_->ApplyForce(hullRot * Vector3::DOWN * Abs(localVelocity.z_) * DOWN_FORCE);
}

void Vehicle::Init()
{
    // This function is called only from the main program when initially creating the vehicle, not on scene load
    ResourceCache* cache = GetSubsystem<ResourceCache>();

    StaticModel* hullObject = node_->CreateComponent<StaticModel>();
    hullBody_ = node_->CreateComponent<RigidBody>();
    CollisionShape* hullShape = node_->CreateComponent<CollisionShape>();

    hullObject->SetModel(cache->GetResource<Model>("Models/APXS.mdl"));
    hullObject->SetMaterial(cache->GetResource<Material>("Materials/DefaultWhite.xml"));
    hullObject->SetCastShadows(true);
    hullShape->SetBox(Vector3(2.0f, 1.0f, 2.0f));
    hullShape->SetPosition(Vector3(0.0f, 1.0f, 0.0f));
    hullBody_->SetMass(8.0f);
    hullBody_->SetLinearDamping(0.2f); // Some air resistance
    hullBody_->SetAngularDamping(1.0f);
    hullBody_->SetCollisionLayer(1);

#if 0
    hullBody_->SetFriction(0.0f);
    hullBody_->SetAnisotropicFriction(Vector3(0,0,0));
    hullBody_->SetRollingFriction(0.0f);
#endif

    InitWheel("FrontLeft", Vector3(-1.0f, .21f, 1.1f), frontLeft_, frontLeftID_);
    InitWheel("FrontRight", Vector3(1.0f, .21f, 1.1f), frontRight_, frontRightID_);

    InitWheel("MidLeft", Vector3(-1.0f, .29f, -0.1f), midLeft_, midLeftID_);
    InitWheel("MidRight", Vector3(1.0f, .29f, -0.1f), midRight_, midRightID_);

    InitWheel("RearLeft", Vector3(-1.0f, .21f, -1.2f), rearLeft_, rearLeftID_);
    InitWheel("RearRight", Vector3(1.0f, .21f, -1.2f), rearRight_, rearRightID_);

    GetWheelComponents();
}

void Vehicle::InitWheel(const String& name, const Vector3& offset, WeakPtr<Node>& wheelNode, unsigned& wheelNodeID)
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();

    // Note: do not parent the wheel to the hull scene node. Instead create it on the root level and let the physics
    // constraint keep it together
    wheelNode = GetScene()->CreateChild(name);
    wheelNode->SetPosition(node_->LocalToWorld(offset));
    // Remember the ID for serialization
    wheelNodeID = wheelNode->GetID();

    StaticModel* wheelObject = wheelNode->CreateComponent<StaticModel>();
    RigidBody* wheelBody = wheelNode->CreateComponent<RigidBody>();
    CollisionShape* wheelShape = wheelNode->CreateComponent<CollisionShape>();
    Constraint* wheelConstraint = wheelNode->CreateComponent<Constraint>();

    if (offset.x_ < 0) 
      wheelObject->SetModel(cache->GetResource<Model>("Models/wheel_03_L.mdl"));
    else
      wheelObject->SetModel(cache->GetResource<Model>("Models/wheel_03_R.mdl"));
    wheelObject->SetMaterial(cache->GetResource<Material>("Materials/DefaultWhite.xml"));
    wheelObject->SetCastShadows(true);
    wheelShape->SetSphere(0.5f);
    wheelBody->SetFriction(2.0f);
    wheelBody->SetMass(1.0f);
    wheelBody->SetLinearDamping(0.5f); // Some air resistance
    wheelBody->SetAngularDamping(0.7f);// Could also use rolling friction
    wheelBody->SetCollisionLayer(1);
    wheelConstraint->SetConstraintType(CONSTRAINT_HINGE);
    wheelConstraint->SetOtherBody(GetComponent<RigidBody>()); // Connect to the hull body
    wheelConstraint->SetWorldPosition(wheelNode->GetPosition()); // Set constraint's both ends at wheel's location
    wheelConstraint->SetAxis(Vector3::LEFT); // Wheel rotates around its local Y-axis
    wheelConstraint->SetOtherAxis(offset.x_ >= 0.0 ? Vector3::RIGHT : Vector3::LEFT); // Wheel's hull axis points either left or right
    wheelConstraint->SetLowLimit(Vector2(-180.0f, 0.0f)); // Let the wheel rotate freely around the axis
    wheelConstraint->SetHighLimit(Vector2(180.0f, 0.0f));
    wheelConstraint->SetDisableCollision(true); // Let the wheel intersect the vehicle hull
}

void Vehicle::GetWheelComponents()
{
    frontLeftAxis_ = frontLeft_->GetComponent<Constraint>();
    frontRightAxis_ = frontRight_->GetComponent<Constraint>();
    rearLeftAxis_ = rearLeft_->GetComponent<Constraint>();
    rearRightAxis_ = rearRight_->GetComponent<Constraint>();

    frontLeftBody_ = frontLeft_->GetComponent<RigidBody>();
    frontRightBody_ = frontRight_->GetComponent<RigidBody>();
    midLeftBody_ = midLeft_->GetComponent<RigidBody>();
    midRightBody_ = midRight_->GetComponent<RigidBody>();
    rearLeftBody_ = rearLeft_->GetComponent<RigidBody>();
    rearRightBody_ = rearRight_->GetComponent<RigidBody>();
}
