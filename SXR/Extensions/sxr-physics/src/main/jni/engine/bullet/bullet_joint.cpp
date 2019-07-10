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
#include "bullet_world.h"
#include "bullet_joint.h"
#include "bullet_sxr_utils.h"
#include "objects/node.h"
#include "util/sxr_log.h"

#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyLink.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionShapes/btEmptyShape.h>
#include <LinearMath/btDefaultMotionState.h>
#include <LinearMath/btScalar.h>
#include <LinearMath/btTransform.h>

#include <math.h>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_inverse.hpp"
#include <glm/gtx/quaternion.hpp>
#include <contrib/glm/gtc/type_ptr.hpp>

namespace sxr {

BulletJoint::BulletJoint(float mass, int numBones)
        : PhysicsJoint(mass, numBones),
          mMultiBody(nullptr),
          mCollider(nullptr),
          mBoneID(0)
{
    mMultiBody = new btMultiBody(numBones, mass, btVector3(0, 0, 0), true, false);
    mMultiBody->setUserPointer(this);
    mMultiBody->setBaseMass(mass);
    mWorld = nullptr;
}

BulletJoint::BulletJoint(BulletJoint* parent, int boneID, float mass)
        : PhysicsJoint(parent, boneID, mass),
          mMultiBody(nullptr),
          mCollider(nullptr),
          mBoneID(boneID)
{
    mMultiBody = static_cast<BulletJoint*>(parent)->getMultiBody();
    mLink = new btMultibodyLink();
    mLink->m_mass = mass;
    mLink->m_parent = parent->getBoneID();
    mMultiBody->getLink(boneID) = mLink;
    mCollider = new btMultiBodyLinkCollider(mMultiBody, boneID);
    mWorld = nullptr;
}

BulletJoint::BulletJoint(btMultiBody* multiBody)
        : PhysicsJoint(multiBody->getNumLinks(), multiBody->getBaseMass()),
          mMultiBody(multiBody),
          mLink(nullptr)
{
    mMultiBody->setUserPointer(this);
    mWorld = nullptr;
}

BulletJoint::BulletJoint(btMultibodyLink* link)
        : PhysicsJoint(link->m_mass, 0),
          mMultiBody(nullptr),
          mLink(link)
{
    mLink->setUserPointer(this);
    mWorld = nullptr;
}

BulletJoint::~BulletJoint() {
    finalize();
}

void BulletJoint::setMass(float mass)
{
    if (mLink != nullptr)
    {
        mLink->m_mass = btScalar(mass));
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
        mLink->m_jointFriction = btScalar(friction));
    }
}

float BulletJoint::getMass()
{
    return mLink ? mLink->m_mass :  mMultiBody->getBaseMass();
}

void BulletJoint::setName(const char* name)
{
    mName = name;
    if (mLink)
    {
        mLink->m_linkName = mName.c_str();
        mLink->m_jointName = mName.c_str();
    }
}

void BulletJoint::finalize()
{
    if (mMultiBody != nullptr)
    {
        if (mLink != nullptr)
        {
            if (mCollider != nullptr)
            {
                mMultiBody->setLinkCollider(mBoneID, nullptr);
                delete mCollider;
            }
            mMultiBody->getLink(mBoneID) = nullptr;
            delete mLink;
            mLink = nullptr;
        }
        else
        {
            if (mCollider != nullptr)
            {
                mMultiBody->setBaseCollider(nullptr);
                delete mCollider;
            }
            delete mMultiBody;
            mMultiBody = nullptr;
        }
    }
}

void BulletJoint::getWorldTransform(btTransform &centerOfMassWorldTrans) const
{
    Transform* trans = owner_object()->transform();
    centerOfMassWorldTrans = convertTransform2btTransform(trans);
}


void BulletJoint::setWorldTransform(const btTransform &centerOfMassWorldTrans) {
    Node* owner = owner_object();
    Transform* trans = owner->transform();
    btTransform aux; getWorldTransform(aux);
    btTransform physicBody = centerOfMassWorldTrans;
    btVector3 pos = physicBody.getOrigin();
    btQuaternion rot = physicBody.getRotation();
    Node* parent = owner->parent();
    float matrixData[16];

    physicBody.getOpenGLMatrix(matrixData);
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
        mCollider->m_collisionShape->setLocalScaling(ownerScale);
    }

    void BulletJoint::updateConstructionInfo()
    {
        if (mCollider == nullptr)
        {
            // This rigid body was not loaded so its construction must be finished
            Collider *collider = (Collider*) owner_object()->getComponent(COMPONENT_TYPE_COLLIDER);
            if (collider)
            {
                mCollider = new btMultiBodyLinkCollider(mMultiBody, mBoneID);
                mCollider->setMotionState(this);
                mCollider->m_collisionShape = convertCollider2CollisionShape(collider);
                mCollider->m_collisionShape->calculateLocalInertia(getMass(), mLink->m_inertiaLocal);
                updateCollisionShapeLocalScaling();
                if (mLink == nullptr)
                {
                    mMultiBody->setBaseCollider(mCollider);
                }
                else
                {
                    mLink->m_collider = mCollider;
                }
            }
            else
            {
                LOGE("PHYSICS: joint %s does not have collider", getName());
            }
        }
        btTransform t;
        getWorldTransform(&t);
    }

}
