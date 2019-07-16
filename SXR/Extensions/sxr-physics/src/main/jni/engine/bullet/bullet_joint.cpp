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
#include <glm/gtx/quaternion.hpp>
#include <contrib/glm/gtc/type_ptr.hpp>

#include "objects/node.h"
#include "objects/components/component_types.h"
#include "objects/components/transform.h"
#include "objects/components/collider.h"
#include "bullet_world.h"
#include "bullet_joint.h"
#include "bullet_sxr_utils.h"
#include "util/sxr_log.h"

#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyLink.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionShapes/btEmptyShape.h>
#include <LinearMath/btDefaultMotionState.h>
#include <LinearMath/btScalar.h>
#include <LinearMath/btVector3.h>
#include <LinearMath/btTransform.h>



namespace sxr {

BulletJoint::BulletJoint(float mass, int numBones)
        : PhysicsJoint(mass, numBones),
          mMultiBody(nullptr),
          mCollider(nullptr),
          mLink(nullptr),
          mBoneID(0),
          mLinksAdded(0),
          mConstraintsAdded(0)
{
    btScalar q0 = -45.0f * M_PI / 180.0f;
    btQuaternion quat0(btVector3(1, 1, 0).normalized(), q0);
    mMultiBody = new btMultiBody(numBones, mass, btVector3(0, 0, 0), true, false);
    mMultiBody->setUserPointer(this);
    mMultiBody->setBaseMass(mass);
    mMultiBody->setCanSleep(false);
    mMultiBody->setHasSelfCollision(false);
    quat0.normalize();
    mMultiBody->setJointPosMultiDof(0, quat0);
    mWorld = nullptr;
}

BulletJoint::BulletJoint(BulletJoint* parent, int boneID, float mass)
        : PhysicsJoint(parent, boneID, mass),
          mMultiBody(nullptr),
          mCollider(nullptr),
          mBoneID(boneID),
          mLinksAdded(0),
          mConstraintsAdded(0)
{
    mMultiBody = static_cast<BulletJoint*>(parent)->getMultiBody();
    mLink = &(mMultiBody->getLink(boneID - 1));
    mLink->m_mass = mass;
    mLink->m_parent = parent->getBoneID();
    mLink->m_userPtr = this;
    mWorld = nullptr;
}

BulletJoint::BulletJoint(btMultiBody* multiBody)
        : PhysicsJoint(multiBody->getNumLinks(), multiBody->getBaseMass()),
          mMultiBody(multiBody),
          mLink(nullptr),
          mLinksAdded(0),
          mConstraintsAdded(0)
{
    mMultiBody->setUserPointer(this);
    mWorld = nullptr;
}

BulletJoint::BulletJoint(btMultibodyLink* link)
        : PhysicsJoint(link->m_mass, 0),
          mMultiBody(nullptr),
          mLink(link),
          mLinksAdded(0),
          mConstraintsAdded(0)
{
    link->m_userPtr = this;
    mWorld = nullptr;
}

BulletJoint::~BulletJoint()
{
    destroy();
}

void BulletJoint::setMass(float mass)
{
    if (mLink != nullptr)
    {
        mLink->m_mass = btScalar(mass);
    }
    else
    {
        mMultiBody->setBaseMass(btScalar(mass));
    }
}

void BulletJoint::setFriction(float friction)
{
    if (mLink != nullptr)
    {
        mLink->m_jointFriction = btScalar(friction);
    }
}

float BulletJoint::getMass() const
{
    return mLink ? mLink->m_mass :  mMultiBody->getBaseMass();
}

void BulletJoint::destroy()
{
    if (mMultiBody != nullptr)
    {
        if (mLink != nullptr)
        {
            if (mCollider != nullptr)
            {
                mLink->m_collider = nullptr;
                delete mCollider;
            }
            mLink = nullptr;
        }
        else if (mCollider != nullptr)
        {
            mMultiBody->setBaseCollider(nullptr);
            delete mCollider;
        }
        delete mMultiBody;
        mMultiBody = nullptr;
    }
}

void BulletJoint::getWorldTransform(btTransform& centerOfMassWorldTrans) const
{
    Transform* trans = owner_object()->transform();
    centerOfMassWorldTrans = convertTransform2btTransform(trans);
    if (mLink == nullptr)
    {
        mMultiBody->setBaseWorldTransform(centerOfMassWorldTrans);
    }
    else if (mCollider)
    {
        mCollider->setWorldTransform(centerOfMassWorldTrans);
    }
}


void BulletJoint::setWorldTransform(const btTransform& centerOfMassWorldTrans)
{
    Node* owner = owner_object();
    Transform* trans = owner->transform();
    btTransform aux; getWorldTransform(aux);
    btTransform physicBody = centerOfMassWorldTrans;
    btVector3 pos = physicBody.getOrigin();
    btQuaternion rot = physicBody.getRotation();
    Node* parent = owner->parent();
    float matrixData[16];

    centerOfMassWorldTrans.getOpenGLMatrix(matrixData);
    glm::mat4 worldMatrix(glm::make_mat4(matrixData));
    if ((parent != nullptr) && (parent->parent() != nullptr))
    {
        glm::mat4 parentWorld(parent->transform()->getModelMatrix(true));
        glm::mat4 parentInverseWorld(glm::inverse(parentWorld));
        glm::mat4 localMatrix;

        localMatrix = parentInverseWorld * worldMatrix;
        trans->setModelMatrix(localMatrix);
    }
    else
    {
        trans->set_position(pos.getX(), pos.getY(), pos.getZ());
        trans->set_rotation(rot.getW(), rot.getX(), rot.getY(), rot.getZ());
    }
    mWorld->markUpdated(this);
}

    void  BulletJoint::updateCollisionShapeLocalScaling()
    {
        btVector3 ownerScale;
        Node* owner = owner_object();
        if (owner)
        {
            Transform* trans = owner->transform();
            ownerScale.setValue(trans->scale_x(),
                                trans->scale_y(),
                                trans->scale_z());
        }
        else
        {
            ownerScale.setValue(1.0f, 1.0f, 1.0f);
        }
        mCollider->getCollisionShape()->setLocalScaling(ownerScale);
    }

    void BulletJoint::updateConstructionInfo(PhysicsWorld* world)
    {
        Node* owner = owner_object();
        mWorld = static_cast<BulletWorld*>(world);
        if (mCollider == nullptr)
        {
            Collider* collider = (Collider*) owner->getComponent(COMPONENT_TYPE_COLLIDER);
            if (collider)
            {
                btVector3 localInertia;
                mCollider = new btMultiBodyLinkCollider(mMultiBody, mBoneID);
                btCollisionShape* shape = convertCollider2CollisionShape(collider);
                mCollider->setCollisionShape(shape);
                updateCollisionShapeLocalScaling();
                if (mLink == nullptr)
                {
                    shape->calculateLocalInertia(getMass(), localInertia);
                    mMultiBody->setBaseCollider(mCollider);
                    mMultiBody->setBaseInertia(localInertia);
                }
                else
                {
                    shape->calculateLocalInertia(getMass(), localInertia);
                    mLink->m_inertiaLocal = localInertia;
                    mLink->m_collider = mCollider;
                }
            }
            else
            {
                LOGE("PHYSICS: joint %s does not have collider", owner_object()->name().c_str());
            }
            if (mLink)
            {
                mLink->m_linkName = owner->name().c_str();
                mLink->m_jointName = owner->name().c_str();
                BulletJoint* j = static_cast<BulletJoint*>(mMultiBody->getUserPointer());
                j->addLink();
            }
            else
            {
                mMultiBody->setBaseName(owner->name().c_str());
            }
        }
        btTransform t;
        getWorldTransform(t);
    }

    void BulletJoint::addLink()
    {
        if (mWorld->isMultiBody())
        {
            BulletJoint* j = static_cast<BulletJoint*> (mMultiBody->getUserPointer());

            if (j != this)
            {
                j->addLink();
                return;
            }
            ++mLinksAdded;
            if (isReady())
            {
                mMultiBody->finalizeMultiDof();
                static_cast<btMultiBodyDynamicsWorld *>(mWorld->getPhysicsWorld())
                        ->addMultiBody(mMultiBody);
            }
        }
    }

    void BulletJoint::addConstraint()
    {
        if (mWorld->isMultiBody())
        {
            BulletJoint* j = static_cast<BulletJoint*> (mMultiBody->getUserPointer());

            if (j != this)
            {
                j->addConstraint();
                return;
            }
            ++mConstraintsAdded;
            if (isReady())
            {
                mMultiBody->finalizeMultiDof();
                static_cast<btMultiBodyDynamicsWorld *>(mWorld->getPhysicsWorld())
                           ->addMultiBody(mMultiBody);
            }
        }
    }

    bool BulletJoint::isReady() const
    {
        return (mConstraintsAdded == mMultiBody->getNumLinks()) &&
               (mLinksAdded == mMultiBody->getNumLinks());
    }

}
