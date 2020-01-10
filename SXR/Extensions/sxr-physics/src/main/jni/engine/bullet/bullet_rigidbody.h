/* Copyright 2015 Samsung Electronics Co., LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef BULLET_RIGIDBODY_H_
#define BULLET_RIGIDBODY_H_

#include "../physics_rigidbody.h"
#include "bullet_world.h"
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <LinearMath/btMotionState.h>


class btDynamicsWorld;

namespace sxr {

class Node;

class BulletRigidBody : public PhysicsRigidBody, btMotionState
{
 public:
    BulletRigidBody(float mass, int collisionGroup);
    BulletRigidBody(btRigidBody *rigidBody);
    virtual ~BulletRigidBody();
    virtual void copy(PhysicsRigidBody* srcBody);
    virtual void onDisable(Node* owner);
    virtual void onEnable(Node* owner);

    virtual const glm::mat4& getColliderTransform() const { return mColliderTransform; }
    virtual const char* getName() const;
    virtual int getCollisionGroup() const { return mCollisionGroup; }
    virtual int getCollisionMask() const { return mCollisionMask; }

    void    getGravity(float *v3) const;
    void    getLinearVelocity(float *v3) const;
    void    getAngularVelocity(float *v3) const;
    void    getAngularFactor(float *v3) const;
    void    getLinearFactor(float *v3) const;
    void    getDamping(float &angular, float &linear) const;
    float   getCcdMotionThreshold() const;
    float   getContactProcessingThreshold() const;
    void    applyCentralForce(float x, float y, float z);
	void    applyForce(float force_x, float force_y, float force_z,
			           float rel_pos_x, float rel_pos_y, float rel_pos_z);
    void    applyCentralImpulse(float x, float y, float z);
    void    applyImpulse(float impulse_x, float impulse_y, float impulse_z,
                         float rel_pos_x, float rel_pos_y, float rel_pos_z);
    void    applyTorque(float x, float y, float z);
    void    applyTorqueImpulse(float x, float y, float z);

    virtual void setName(const char*);
    virtual void setSimulationType(SimulationType type);
    virtual void setFriction(float f);
    virtual void sync(int options = 0);
    virtual void getWorldTransform(btTransform &worldTrans) const;
    virtual void setWorldTransform(const btTransform &worldTrans);
    virtual void setColliderTransform(const glm::mat4& m);

    void    setGravity(float x, float y, float z);
    void    setDamping(float linear, float angular);
    void    setLinearVelocity(float x, float y, float z);
    void    setAngularVelocity(float x, float y, float z);
    void    setAngularFactor(float x, float y, float z);
    void    setLinearFactor(float x, float y, float z);
    void    setRestitution(float n);
    void    setMass(float mass);
    void    setSleepingThresholds(float linear, float angular);
    void    setCcdMotionThreshold(float n);
    void    setCcdSweptSphereRadius(float n);
    void    setContactProcessingThreshold(float n);
    void    setIgnoreCollisionCheck(PhysicsRigidBody *collisionObj, bool ignore);


    float   getCcdSweptSphereRadius() const;
    void    addToWorld(PhysicsWorld* world);
    void    addToWorld(PhysicsWorld* world, int collidesWith);

    btRigidBody* getRigidBody() const
    {
        return mRigidBody;
    }

    btDynamicsWorld* getPhysicsWorld() { return mWorld ? mWorld->getPhysicsWorld() : nullptr; }

    virtual void* getUnderlying() const
    {
        return mRigidBody;
    }

    virtual SimulationType getSimulationType() const
    {
        return mSimType;
    }

    virtual float getMass() const
    {
        return mConstructionInfo.m_mass;
    }

    virtual float getFriction() const
    {
        return mConstructionInfo.m_friction;
    }

    float getRestitution() const
    {
        return mConstructionInfo.m_restitution;
    }

protected:
    bool updateCollider(Node* owner, int options);
    void finalize();
    void updateSimType();

private:
    btRigidBody::btRigidBodyConstructionInfo mConstructionInfo;
    btRigidBody*        mRigidBody;
    SimulationType      mSimType;
    BulletWorld*        mWorld;
    glm::mat4           mColliderTransform;
    int                 mNeedsSync;
    int 				mCollisionGroup;
    int 				mCollisionMask;
    mutable std::string mName;
};

}

#endif /* BULLET_RIGIDBODY_H_ */
