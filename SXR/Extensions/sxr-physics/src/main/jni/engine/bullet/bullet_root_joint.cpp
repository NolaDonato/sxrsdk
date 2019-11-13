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

    BulletRootJoint::BulletRootJoint(float mass, int numJoints)
    : BulletJoint(mass, numJoints),
      mNumJoints(numJoints - 1),
      mLinksAdded(0)
    {
        mJointType = JointType::baseJoint;
        mJoints.reserve(mNumJoints);
        mJoints.resize(mNumJoints);
    }

    BulletRootJoint::BulletRootJoint(btMultiBody* multiBody)
    : BulletJoint(multiBody->getNumLinks(), multiBody->getBaseMass()),
      mLinksAdded(0),
      mNumJoints(multiBody->getNumLinks())
    {
        mMultiBody = multiBody;
        mJoints.reserve(mNumJoints);
        mJoints.resize(mNumJoints);
        mMultiBody->setUserPointer(this);
        mCollider = mMultiBody->getBaseCollider();
        if (mCollider)
        {
            mCollider->setUserPointer(this);
        }
        for (int i = 0; i < mNumJoints; ++i)
        {
            btMultibodyLink& link = mMultiBody->getLink(i);
            BulletJoint* parent = (link.m_parent >= 0) ? mJoints[link.m_parent] : this;
            mJoints[i] = new BulletJoint(parent, i);
            link.m_userPtr = mJoints[i];
            if (link.m_collider)
            {
                link.m_collider->setUserPointer(mJoints[i]);
            }
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

    Skeleton* BulletRootJoint::getSkeleton() const
    {
        Skeleton* skel = static_cast<Skeleton*>(owner_object()->getComponent(COMPONENT_TYPE_SKELETON));
        if (skel == nullptr)
        {
            skel = createSkeleton();
            owner_object()->attachComponent(skel);
        }
        return skel;
    }

    int BulletRootJoint::getNumJoints() const { return mNumJoints + 1; }

    void BulletRootJoint::setNumJoints(int n)
    {
        if (mMultiBody)
        {
            detachFromWorld(true);
            delete mMultiBody;
            mMultiBody = nullptr;
        }
        mNumJoints = n;
        mJoints.resize(n);
    }

    int BulletRootJoint::addJointToBody(PhysicsJoint* child)
    {
        BulletJoint* childJoint = static_cast<BulletJoint*>(child);
        int nextJointIndex = getNumJoints();

        if (childJoint->getMultiBody())
        {
            childJoint->removeJointFromBody(childJoint->getJointIndex());
        }
        setNumJoints(nextJointIndex + 1);
        childJoint->update(nextJointIndex, this);
        LOGD("BULLET: adding joint %s at index %d to body",child->getName(), nextJointIndex);
        return nextJointIndex;
    }

    void BulletRootJoint::removeJointFromBody(int jointIndex)
    {
        int startIndex = jointIndex + 1;

        if ((jointIndex < 0) || (startIndex >= mNumJoints))
        {
            return;
        }
        BulletJoint* bj = getJoint(jointIndex);
        BulletJoint* parent = (BulletJoint*) bj->getParent();

        if (bj == nullptr)
        {
            return;
        }
        removeJointFromWorld(bj);
        bj->update(0, nullptr);
        for (auto it = mJoints.begin() + startIndex; it != mJoints.end(); ++it)
        {
            bj = *it;
            if (bj != nullptr)
            {
                BulletJoint *bp = static_cast<BulletJoint *>(bj->getParent());
                if (mMultiBody)
                {
                    btMultibodyLink& link = mMultiBody->getLink(bj->getJointIndex());
                    if (link.m_parent == jointIndex)
                    {
                        bp = parent;
                        link.m_parent = bp->getJointIndex();
                    }
                    else if (link.m_parent >= startIndex)
                    {
                        link.m_parent--;
                    }
                }
                bj->update(bj->getJointIndex() - 1, bp);
            }
        }
        mJoints.erase(mJoints.begin() + jointIndex);
        setNumJoints(mNumJoints - 1);
        LOGD("BULLET: removing joint %s at index %d from body", bj->getName(), jointIndex);
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
        if (mMultiBody)
        {
            getPhysicsTransform();
            for (int j = 0; j < mNumJoints; ++j)
            {
                BulletJoint* joint = mJoints[j];

                if (joint->enabled())
                {
                    joint->getPhysicsTransform();
                }
            }
        }
    }

    void BulletRootJoint::setPhysicsTransforms()
    {
        if (mMultiBody)
        {
            setPhysicsTransform();
            for (int j = 0; j < mNumJoints; ++j)
            {
                BulletJoint* joint = mJoints[j];
                if (joint->enabled())
                {
                    joint->setPhysicsTransform();
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
            *(++curParent) = j->getParent()->getJointIndex();
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
        updateCollider(owner_object(), options);
        if (options & SyncOptions::TRANSFORM)
        {
            setPhysicsTransform();
        }
    }

    void BulletRootJoint::updateCollider(Node* owner, int options)
    {
        btCollisionShape* oldShape = nullptr;
        btCollisionShape* newShape = nullptr;
        btVector3 ownerScale;
        Transform* trans = owner->transform();
        btVector3 localInertia;
        Collider* collider = (Collider*) owner->getComponent(COMPONENT_TYPE_COLLIDER);

        if (collider == nullptr)
        {
            return;
        }
        newShape = convertCollider2CollisionShape(collider);
        if (mCollider && ((options & SyncOptions::COLLISION_SHAPE) != 0))
        {
            oldShape = mCollider->getCollisionShape();
            mCollider->setCollisionShape(newShape);
            if (oldShape)
            {
                delete oldShape;
            }
        }
        else
        {
            mCollider = new btMultiBodyLinkCollider(mMultiBody, mJointIndex);
            LOGV("BULLET: creating base collider %s", getName());
            mCollider->setCollisionShape(newShape);
            mCollider->m_link = getJointIndex();
            mCollider->setUserPointer(this);
            mMultiBody->setBaseCollider(mCollider);
        }
        ownerScale.setValue(trans->scale_x(), trans->scale_y(), trans->scale_z());
        newShape->setLocalScaling(ownerScale);
        newShape->calculateLocalInertia(getMass(), localInertia);
        mMultiBody->setBaseInertia(localInertia);
    }


    bool BulletRootJoint::addJointToWorld(PhysicsJoint* joint, PhysicsWorld* world)
    {
        int linkIndex = joint->getJointIndex();
        int numjoints = mNumJoints + 1;

        if (mLinksAdded == numjoints)
        {
            return false;
        }
        BulletJoint* bj = static_cast<BulletJoint*>(joint);

        LOGD("BULLET: linking joint %s at index %d", joint->getName(), linkIndex);
        if (joint != this)
        {
            mJoints[linkIndex] = bj;
        }
        if (bj->getMultiBody())
        {
            bj->getPhysicsTransform();
            bj->attachToWorld(world);
            return ++mLinksAdded == numjoints;
        }
        if (++mLinksAdded == numjoints)
        {
            attachToWorld(world);
            return true;
        }
        return false;
    }

    bool BulletRootJoint::removeJointFromWorld(PhysicsJoint* joint)
    {
        LOGD("BULLET: unlinking joint %s at index %d", joint->getName(), joint->getJointIndex());
        if (mLinksAdded <= 0)
        {
            return mNumJoints == 0;
        }
        if (joint != this)
        {
            static_cast<BulletJoint*>(joint)->detachFromWorld();
        }
        if (--mLinksAdded == 0)
        {
            detachFromWorld();
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
            mMultiBody->setCanSleep(false);
            mMultiBody->setHasSelfCollision(false);
            mMultiBody->setBaseMass(mMass);
        }
        sync(SyncOptions::PROPERTIES | SyncOptions::TRANSFORM);
        for (int i = 0; i < mNumJoints; ++i)
        {
            BulletJoint* joint = mJoints[i];
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
        mWorld = static_cast<BulletWorld*>(world);
        btMultiBodyDynamicsWorld* bw = dynamic_cast<btMultiBodyDynamicsWorld*>(mWorld->getPhysicsWorld());
        Node* owner = owner_object();

        if (owner && !owner->name().empty())
        {
            mName = owner->name();
        }
        bw->addCollisionObject(mCollider, mCollisionGroup, mCollisionMask);
        mMultiBody->setBaseName(mName.c_str());
        bw->addMultiBody(mMultiBody, mCollisionGroup, mCollisionMask);
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
