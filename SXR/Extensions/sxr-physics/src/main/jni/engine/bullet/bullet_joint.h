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

#ifndef BULLET_JOINT_H_
#define BULLET_JOINT_H_

#include <string>
#include "glm/mat4x4.hpp"
#include "../physics_joint.h"
#include "bullet_world.h"
#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyLink.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <LinearMath/btMotionState.h>

class btDynamicsWorld;
class btMultiBodyJointMotor;
class btMultiBodyConstraint;

namespace sxr {

class Node;
class Skeleton;
class BulletWorld;
class BulletHingeConstraint;
class BulletSliderConstraint;
class BulletFixedConstraint;
class BulletGeneric6dofConstraint;
class BulletRootJoint;

class BulletJoint : public PhysicsJoint
{
 public:
    BulletJoint(BulletJoint* parent, JointType type, int jointIndex, float mass);

	BulletJoint(BulletJoint* parent, int jointIndex);
    virtual ~BulletJoint() { }

	virtual const BulletRootJoint* findRoot() const;
	virtual BulletRootJoint* findRoot();
	virtual const char*      getName() const;
	virtual Skeleton*        getSkeleton() const;
	virtual float 			 getLinearDamping() const { return mLinearDamping; }
	virtual float 			 getAngularDamping() const { return mAngularDamping; }
	virtual float            getMaxAppliedImpulse() const { return mMaxAppliedImpulse; }
	virtual float            getMaxCoordVelocity() const { return mMaxCoordVelocity; }
	virtual const glm::vec3& getScale() const { return mScale; }

	virtual void  setName(const char*);
	virtual void  setMass(float mass);
	virtual void  setLinearDamping(float ld);
	virtual void  setAngularDamping(float ad);
	virtual void  setMaxAppliedImpulse(float v);
	virtual void  setMaxCoordVelocity(float v);
	virtual void  setScale(const glm::vec3& v);
	virtual float getFriction() const;
	virtual void  setFriction(float f);
	virtual void  applyCentralForce(float x, float y, float z);
	virtual void  applyTorque(float x, float y, float z);
	virtual void  applyTorque(float t);
	virtual void  getWorldTransform(btTransform& t);
	virtual void  setPhysicsTransform();
	virtual void  getPhysicsTransform();
	virtual void  sync(int options = 0);
	virtual void  detachFromWorld(bool deleteCollider = false);
	virtual void  attachToWorld(PhysicsWorld* world);
	virtual int   getNumJoints() const;
	virtual int   addJointToBody(PhysicsJoint* newJoint);
	virtual void  removeJointFromBody(int jointIndex);
    virtual void  onDisable(Node* owner);
    virtual void  onEnable(Node* owner);

    void          setCollisionProperties(int collisionGroup, int collidesWith);
    void          update(int jointIndex, BulletJoint* parent);
    bool 		  isImported() const { return mNeedsSync & SyncOptions::IMPORTED; }

    btDynamicsWorld* 	     getPhysicsWorld() { return mWorld ? mWorld->getPhysicsWorld() : nullptr; }
	btMultiBody*             getMultiBody() const { return mMultiBody; }
	btMultibodyLink*         getLink() const { return &(mMultiBody->getLink(mJointIndex)); }
	int                      getJointIndex() const { return mJointIndex; }
	virtual float            getMass() const { return mMass; }
    virtual JointType        getJointType() const { return mJointType; }
    virtual PhysicsJoint*    getParent() const { return mParent; }
    virtual const glm::vec3& getAxis() const { return mAxis; }
    virtual const glm::vec3& getPivot() const { return mPivot; }
	virtual void             setPivot(const glm::vec3& pivot);
    virtual void             setAxis(const glm::vec3& axis);

protected:
    BulletJoint(float mass, int numBones);
    virtual void updateCollider(Node* owner, int options);
    void setupSpherical();
    void setupHinge();
    void setupSlider();
    void setupFixed();
    void updateFixed();
    void updateSpherical();
    void updateHinge();
    void updateSlider();

protected:
	mutable std::string		 mName;
    BulletWorld*             mWorld;
    BulletJoint*             mParent;
    btMultiBodyLinkCollider* mCollider;
    btMultiBody*             mMultiBody;
    JointType                mJointType;
    glm::vec3                mAxis;
    glm::vec3                mPivot;
	glm::vec3				 mScale;
	glm::vec3				 mLocalInertia;
	short 				     mNeedsSync;
	short                    mJointIndex;
    int 					 mCollisionGroup;
    int 					 mCollisionMask;
	float                    mMass;
	float                    mFriction;
    float 					 mLinearDamping;
    float 					 mAngularDamping;
    float                    mMaxAppliedImpulse;
    float                    mMaxCoordVelocity;
};

class BulletRootJoint : public BulletJoint
{
public:
    BulletRootJoint(float mass, int numBones);
    BulletRootJoint(btMultiBody* multibody);
    virtual ~BulletRootJoint();

	virtual const BulletRootJoint* findRoot() const;
	virtual BulletRootJoint* findRoot();
    virtual Skeleton*        getSkeleton() const;
    virtual BulletJoint*     getJoint(int jointIndex) { return mJoints[jointIndex]; }
	virtual int  getNumJoints() const;

    virtual void setMass(float mass);
    virtual void setLinearDamping(float ld);
    virtual void setAngularDamping(float ad);
    virtual void setMaxAppliedImpulse(float v);
    virtual void setMaxCoordVelocity(float v);

    virtual void applyCentralForce(float x, float y, float z);
    virtual void applyTorque(float x, float y, float z);
    virtual void applyTorque(float t);
    virtual void setPhysicsTransform();
	virtual void detachFromWorld(bool deleteCollider = false);
	virtual void attachToWorld(PhysicsWorld* world);
	virtual void sync(int options = 0);
	virtual int  addJointToBody(PhysicsJoint* newJoint);
	virtual void removeJointFromBody(int jointIndex);
	bool	     addJointToWorld(PhysicsJoint* joint, PhysicsWorld* world);
	bool	     removeJointFromWorld(PhysicsJoint* joint, bool deleteCollider);
	void         setPhysicsTransforms();
	void	     getPhysicsTransforms();

protected:
    virtual void updateCollider(Node* owner, int options);
    void         destroy();
    Skeleton*    createSkeleton() const;

    std::vector<BulletJoint*> mJoints;
    Skeleton*   mSkeleton;
    int         mNumJoints;
    int         mLinksAdded;
};
}

#endif /* BULLET_JOINT_H_ */
