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
#include <math.h>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_inverse.hpp"
#include "glm/gtx/quaternion.hpp"
#include "glm/mat4x4.hpp"
#include "glm/gtc/type_ptr.hpp"

#include "objects/node.h"
#include "objects/components/skeleton.h"
#include "objects/components/component_types.h"
#include "objects/components/transform.h"
#include "objects/components/collider.h"
#include "bullet_world.h"
#include "bullet_joint.h"
#include "bullet_hingeconstraint.h"
#include "bullet_sliderconstraint.h"
#include "bullet_fixedconstraint.h"
#include "bullet_generic6dofconstraint.h"
#include "bullet_jointmotor.h"
#include "bullet_sxr_utils.h"
#include "util/sxr_log.h"

#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyLink.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"

#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btEmptyShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "LinearMath/btDefaultMotionState.h"
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btMatrix3x3.h"


namespace sxr {

    BulletRootJoint::BulletRootJoint(float mass, int numJoints, int collisionGroup)
    : BulletJoint(mass, numJoints, collisionGroup),
      mNumJoints(numJoints - 1),
      mLinksAdded(0),
      mMaxCoordVelocity(100),
      mMaxAppliedImpulse(1000)
    {
        mJointType = JointType::baseJoint;
        mJoints.reserve(mNumJoints);
        mJoints.resize(mNumJoints);
    }

    BulletRootJoint::BulletRootJoint(btMultiBody* multiBody)
    : BulletJoint(multiBody->getNumLinks(), multiBody->getBaseMass(),
                  btBroadphaseProxy::DefaultFilter),
      mLinksAdded(0),
      mMaxCoordVelocity(100),
      mMaxAppliedImpulse(1000),
      mNumJoints(multiBody->getNumLinks())
    {
        mNeedsSync |= SyncOptions::IMPORTED | SyncOptions::COLLISION_SHAPE;
        mMultiBody = multiBody;
        mJoints.reserve(mNumJoints);
        mJoints.resize(mNumJoints);
        mMultiBody->setUserPointer(this);
        mCollider = mMultiBody->getBaseCollider();
        mLinearDamping = mMultiBody->getLinearDamping();
        mAngularDamping = mMultiBody->getAngularDamping();
        mCollider = multiBody->getBaseCollider();
        mMaxAppliedImpulse = multiBody->getMaxAppliedImpulse();
        mMaxCoordVelocity = multiBody->getMaxCoordinateVelocity();
        if (mCollider)
        {
            btCollisionShape* shape = mCollider->getCollisionShape();
            if (shape)
            {
                btVector3 scale = shape->getLocalScaling();
                mScale.x = scale.x();
                mScale.y = scale.y();
                mScale.z = scale.z();
            }
            mCollider->setUserPointer(this);
            mCollisionGroup = mCollider->getBroadphaseHandle()->m_collisionFilterGroup;
        }
        for (int i = 0; i < mNumJoints; ++i)
        {
            btMultibodyLink& link = mMultiBody->getLink(i);
            BulletJoint* parent = (link.m_parent >= 0) ? mJoints[link.m_parent] : this;
            int collisionGroup = btBroadphaseProxy::DefaultFilter;

            if (link.m_collider)
            {
                collisionGroup = link.m_collider->getBroadphaseHandle()->m_collisionFilterGroup;
                mJoints[i] = new BulletJoint(parent, i, collisionGroup);
                link.m_collider->setUserPointer(mJoints[i]);
            }
            else
            {
                mJoints[i] = new BulletJoint(parent, i, collisionGroup);
            }
            link.m_userPtr = mJoints[i];
        }
    }

    BulletRootJoint::~BulletRootJoint()
    {
        destroy();
    }

    const BulletRootJoint* BulletRootJoint::findRoot() const
    {
        return this;
    }

    BulletRootJoint* BulletRootJoint::findRoot()
    {
        return this;
    }

    void BulletRootJoint::copy(PhysicsJoint* srcJoint)
    {
        BulletJoint* src = static_cast<BulletJoint*>(srcJoint);
        
        mMaxCoordVelocity = src->getMaxCoordVelocity();
        mMaxAppliedImpulse = src->getMaxAppliedImpulse();
        BulletJoint::copy(srcJoint);
    }

    Skeleton* BulletRootJoint::getSkeleton() const
    {
        Node* owner = owner_object();

        if (owner != nullptr)
        {
            Skeleton* skel = static_cast<Skeleton *>(owner_object()->getComponent(COMPONENT_TYPE_SKELETON));
            if (skel == nullptr)
            {
                skel = createSkeleton();
                owner->attachComponent(skel);
            }
        }
        return createSkeleton();
    }

    int BulletRootJoint::getNumJoints() const { return mNumJoints + 1; }

    void BulletRootJoint::setLinearDamping(float ld)
    {
        mLinearDamping = ld;
        if (mMultiBody)
        {
            mMultiBody->setLinearDamping(ld);
        }
    }

    void BulletRootJoint::setAngularDamping(float ad)
    {
        mAngularDamping = ad;
        if (mMultiBody)
        {
            mMultiBody->setAngularDamping(ad);
        }
    }

    void BulletRootJoint::setMaxAppliedImpulse(float v)
    {
        mMaxAppliedImpulse = v;
        if (mMultiBody)
        {
            mMultiBody->setMaxAppliedImpulse(v);
        }
    }

    void BulletRootJoint::setMaxCoordVelocity(float v)
    {
        mMaxCoordVelocity = v;
        if (mMultiBody)
        {
            mMultiBody->setMaxCoordinateVelocity(v);
        }
    }

    int BulletRootJoint::addJointToBody(PhysicsJoint* child)
    {
        BulletJoint* childJoint = static_cast<BulletJoint*>(child);
        int nextJointIndex = getNumJoints() - 1;

        mJoints.resize(++mNumJoints);
        mJoints[nextJointIndex] = childJoint;
        if (mMultiBody)
        {
            mMultiBody->setNumLinks(nextJointIndex + 1);
        }
        childJoint->update(nextJointIndex, this);
        LOGD("BULLET: adding joint %s at index %d to body", child->getName(), nextJointIndex);
        return nextJointIndex;
    }

    void BulletRootJoint::removeJointFromBody(int jointIndex)
    {
        if (jointIndex >= mNumJoints)
        {
            return;
        }
        BulletJoint* bj = (--jointIndex < 0) ? this : getJoint(jointIndex);
        btMultiBodyDynamicsWorld *bw = nullptr;
        BulletJoint* parent = (BulletJoint*) bj->getParent();

        if (bj)
        {
            LOGD("BULLET: removing joint %s at index %d from body", bj->getName(), jointIndex);
            removeJointFromWorld(bj, true);
            bj->update(0, nullptr);
        }
        if (mWorld)
        {
            bw = dynamic_cast<btMultiBodyDynamicsWorld*>(mWorld->getPhysicsWorld());
            if (mMultiBody)
            {
                bw->removeMultiBody(mMultiBody);
            }
        }
        btMultiBody* mb = new btMultiBody(mNumJoints - 1,
                                          mMultiBody->getBaseMass(),
                                          mMultiBody->getBaseInertia(),
                                          mMultiBody->hasFixedBase(),
                                          mMultiBody->getCanSleep());
        mb->setUserPointer(this);
        mb->setHasSelfCollision(mMultiBody->hasSelfCollision());
        mb->setBaseCollider(mMultiBody->getBaseCollider());
        mb->setBaseWorldTransform(mMultiBody->getBaseWorldTransform());
        mb->setBaseName(getName());
        mb->setLinearDamping(mMultiBody->getLinearDamping());
        mb->setAngularDamping(mMultiBody->getAngularDamping());
        mb->setMaxAppliedImpulse(mMultiBody->getMaxAppliedImpulse());
        mb->setMaxCoordinateVelocity(mMultiBody->getMaxCoordinateVelocity());
        mb->finalizeMultiDof();
        mJoints.erase(mJoints.begin() + jointIndex);
        --mNumJoints;
        for (auto it = mJoints.begin() + jointIndex; it != mJoints.end(); ++it)
        {
            bj = *it;
            if (bj != nullptr)
            {
                BulletJoint* bp = static_cast<BulletJoint*>(bj->getParent());
                int newIndex = bj->getJointIndex() - 1;

                if (bp->getJointIndex() == jointIndex)
                {
                    bp = parent;
                }
                bj->update(newIndex, bp);
            }
        }
        if (bw)
        {
            mMultiBody = mb;
            bw->addMultiBody(mb, mCollisionGroup, mCollisionMask);
        }
    }

    void BulletRootJoint::setMass(float mass)
    {
        mMass = mass;
        if (mMultiBody)
        {
            mMultiBody->setBaseMass(mass);
        }
    }

    void BulletRootJoint::destroy()
    {
        mWorld = nullptr;
        if (mMultiBody != nullptr)
        {
            int numLinks = mMultiBody->getNumLinks();

            for (int i = 0; i < numLinks; ++i)
            {
                btMultibodyLink& link = mMultiBody->getLink(i);
                if (link.m_collider)
                {
                    delete link.m_collider;
                    link.m_collider = nullptr;
                }
            }
            delete mMultiBody;
            mMultiBody = nullptr;
        }
    }

    void BulletRootJoint::setPhysicsTransform()
    {
        Node* owner = owner_object();
        Transform* trans = owner->transform();
        btTransform  t = convertTransform2btTransform(trans);
        btVector3 pos = t.getOrigin();

        LOGE("BULLET ROOT JOINT: UPDATE %s, %f, %f, %f", owner->name().c_str(), pos.x(), pos.y(), pos.z());
        if (mMultiBody)
        {
            mMultiBody->setBaseWorldTransform(t);
        }
        if (mCollider)
        {
            mCollider->setWorldTransform(t);
        }
    }

    void BulletRootJoint::getPhysicsTransforms()
    {
        if (mMultiBody && enabled())
        {
            sync();
            getPhysicsTransform();
            for (int j = 0; j < mNumJoints; ++j)
            {
                BulletJoint* joint = mJoints[j];

                if (joint->enabled())
                {
                    joint->sync();
                    joint->getPhysicsTransform();
                }
            }
        }
    }

    void BulletRootJoint::setPhysicsTransforms()
    {
        if (mMultiBody && enabled())
        {
            sync(SyncOptions::TRANSFORM);
            for (int j = 0; j < mNumJoints; ++j)
            {
                BulletJoint* joint = mJoints[j];
                if (joint->enabled())
                {
                    joint->sync(SyncOptions::TRANSFORM);
                }
            }
        }
    }

    Skeleton* BulletRootJoint::createSkeleton() const
    {
        int          numbones = mMultiBody->getNumLinks() + 1;
        int*         boneParents = new int[numbones];
        int*         curParent = boneParents;
        Skeleton*    skel;

        *curParent = -1;
        for (int i = 0; i < mNumJoints; ++i)
        {
            BulletJoint* j = mJoints[i];
            *(++curParent) = j->getParent()->getJointIndex() + 1;
        }
        skel = new Skeleton(boneParents, numbones);
        delete [] boneParents;
        skel->setBoneName(0, getName());
        for (int i = 0; i < mNumJoints; ++i)
        {
            BulletJoint* j = mJoints[i];
            const char* name = j->getName();
            skel->setBoneName(i + 1, name);
        }
        return skel;
    }

    void BulletRootJoint::applyCentralForce(float x, float y, float z)
    {
        btVector3 force(x, y, z);
        if (mMultiBody)
        {
            mMultiBody->addBaseForce(force);
        }
    }

    void BulletRootJoint::applyTorque(float x, float y, float z)
    {
        if (mMultiBody)
        {
            btVector3 torque(x, y, z);
            mMultiBody->addBaseTorque(torque);
        }
    }

    void BulletRootJoint::applyTorque(float t)
    {
        if (mMultiBody)
        {
            btVector3 torque(t, 0, 0);
            mMultiBody->addBaseTorque(torque);
        }
    }

    void BulletRootJoint::sync(int options)
    {
        options |= mNeedsSync;
        mNeedsSync = 0;
        updateCollider(owner_object(), options);
        if (options & SyncOptions::TRANSFORM)
        {
            setPhysicsTransform();
        }
    }

    void BulletRootJoint::updateCollider(Node* owner, int options)
    {
        Collider* collider = (Collider*) owner->getComponent(COMPONENT_TYPE_COLLIDER);

        if (collider == nullptr)
        {
            return;
        }
        if (mCollider == nullptr)
        {
            mCollider = new btMultiBodyLinkCollider(mMultiBody, mJointIndex);
            LOGV("BULLET: creating link collider %s", getName());
            mMultiBody->setBaseCollider(mCollider);
            mCollider->setUserPointer(this);
            options |= SyncOptions::PROPERTIES;
        }
        BulletJoint::updateCollider(owner, options);
    }

    bool BulletRootJoint::addJointToWorld(PhysicsJoint* joint, PhysicsWorld* world)
    {
        int numjoints = mNumJoints + 1;

        if (mLinksAdded == numjoints)
        {
            return false;
        }
        BulletJoint* bj = static_cast<BulletJoint*>(joint);
        int          linkIndex = joint->getJointIndex();
        bool         initialized = (++mLinksAdded == numjoints);

        LOGD("BULLET: linking joint %s at index %d", joint->getName(), linkIndex);

        if (bj->isImported())
        {
            if (joint != this)
            {
                bj->attachToWorld(world);
            }
        }
        else
        {
            if (joint != this)
            {
                mJoints[linkIndex] = bj;
            }
            if (initialized)
            {
                attachToWorld(world);
            }
        }
        return initialized;
    }

    bool BulletRootJoint::removeJointFromWorld(PhysicsJoint* joint, bool deleteCollider)
    {
        if (mLinksAdded <= 0)
        {
            return mNumJoints == 0;
        }
        LOGD("BULLET: unlinking joint %s at index %d", joint->getName(), joint->getJointIndex());
        if (joint != this)
        {
            static_cast<BulletJoint*>(joint)->detachFromWorld(deleteCollider);
        }
        --mLinksAdded;
        if (mWorld)
        {
            detachFromWorld(deleteCollider);
            return true;
        }
        return false;
    }

    void BulletRootJoint::attachToWorld(PhysicsWorld* world)
    {
        bool createMB = (mMultiBody == nullptr);

        if (createMB)
        {
            mMultiBody = new btMultiBody(mNumJoints,
                                         mMass,
                                         btVector3(0, 0, 0),
                                         (mMass == 0),
                                         false);
            mMultiBody->setUserPointer(this);
            mMultiBody->setHasSelfCollision(false);
            mMultiBody->setBaseMass(mMass);
            mMultiBody->setLinearDamping(mLinearDamping);
            mMultiBody->setAngularDamping(mAngularDamping);
            mMultiBody->setMaxAppliedImpulse(mMaxAppliedImpulse);
            mMultiBody->setMaxCoordinateVelocity(mMaxCoordVelocity);
        }
        if (mNeedsSync & SyncOptions::IMPORTED)
        {
            getPhysicsTransform();
            updateCollider(owner_object(), mNeedsSync);
            mNeedsSync = 0;
        }
        else
        {
            sync(SyncOptions::PROPERTIES | SyncOptions::TRANSFORM);
            for (int i = 0; i < mNumJoints; ++i)
            {
                BulletJoint *joint = mJoints[i];
                if (joint != nullptr)
                {
                    joint->sync(SyncOptions::PROPERTIES | SyncOptions::TRANSFORM);
                    joint->attachToWorld(world);
                }
            }
            if (createMB)
            {
                mMultiBody->finalizeMultiDof();
            }
        }
        mWorld = static_cast<BulletWorld*>(world);
        btMultiBodyDynamicsWorld* bw = dynamic_cast<btMultiBodyDynamicsWorld*>(mWorld->getPhysicsWorld());
        Node* owner = owner_object();

        if (owner && !owner->name().empty())
        {
            mName = owner->name();
        }
        bw->addCollisionObject(mCollider, mCollisionGroup, mCollisionMask);
        mMultiBody->setBaseName(mName.c_str());
        bw->addMultiBody(mMultiBody);
        LOGD("BULLET: attaching root joint %s to world", getName());
    }

    void BulletRootJoint::detachFromWorld(bool deleteCollider)
    {
        if (mWorld && mMultiBody)
        {
            btMultiBodyDynamicsWorld* w = dynamic_cast<btMultiBodyDynamicsWorld*>(mWorld->getPhysicsWorld());
            BulletJoint::detachFromWorld(deleteCollider);
            mWorld = nullptr;
            w->removeMultiBody(mMultiBody);
            LOGD("BULLET: detaching root joint %s from world", getName());
        }
    }
}
