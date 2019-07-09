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
#include "bullet_rigidbody.h"
#include "bullet_sxr_utils.h"
#include "objects/node.h"
#include "objects/components/sphere_collider.h"
#include "util/sxr_log.h"

#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyLink.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionShapes/btEmptyShape.h>
#include <LinearMath/btDefaultMotionState.h>
#include <LinearMath/btTransform.h>
#include <math.h>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_inverse.hpp"
#include "bullet_joint.h"
#include <glm/gtx/quaternion.hpp>
#include <contrib/glm/gtc/type_ptr.hpp>

namespace sxr {

BulletJoint::BulletJoint(float mass, int boneID, PhysicsJoint* parent)
        : PhysicsJoint(mass, boneID, parent),
          mMultiBody(nullptr),
          mBoneID(boneID),
          mLink(nullptr)
{
    if (parent == nullptr)
    {
        mMultiBody = new btMultiBody(0, mass, btVector3(0, 0, 0), true, false);
        mMultiBody->setUserPointer(this);
    }
    else
    {
        mMultiBody = static_cast<BulletJoint*>(parent)->getMultiBody();
        mLink = new btMultibodyLink();
        mLink->m_mass = mass;
    }
    mWorld = nullptr;
}

BulletJoint::BulletJoint(btMultiBody* multiBody)
        : PhysicsJoint(multiBody->getBaseMass(), 0, nullptr),
          mMultiBody(multiBody),
          mLink(nullptr)
{
    mMultiBody->setUserPointer(this);
    mWorld = nullptr;
}

BulletJoint::BulletJoint(btMultibodyLink* link)
        : PhysicsJoint(link->m_mass, 0),
          mMultiBody(link),
          mLink(nullptr)
{
    mMultiBody->setUserPointer(this);
    mWorld = nullptr;
}


BulletJoint::~BulletJoint() {
    finalize();
}

void BulletJoint::setMass(float mass) {  mMultiBody->setBaseMass(btScalar(mass)); }

float BulletJoint::getMass() {return mMultiBody->getBaseMass(); }


void BulletJoint::finalize()
{
    if (mMultiBody != nullptr)
    {
        if (mLink != nullptr)
        {
            btMultiBodyLinkCollider* lc = mMultiBody->getLinkCollider(mBoneID);
            if (lc != nullptr)
            {
                mMultiBody->setLinkCollider(mBoneID, nullptr);
                delete lc;
            }
            delete mLink;
            mLink = nullptr;
        }
        else
        {
            btMultiBodyLinkCollider* bc = mMultiBody->getBaseCollider();
            if (bc != nullptr)
            {
                mMultiBody->setBaseCollider(nullptr);
                delete bc;
            }
            delete mMultiBody;
            mMultiBody = nullptr;
        }
    }
}

void BulletJoint::getWorldTransform(btTransform &centerOfMassWorldTrans) const {
    Transform* trans = owner_object()->transform();

    centerOfMassWorldTrans = convertTransform2btTransform(trans);
}

void BulletJoint::setWorldTransform(const btTransform &centerOfMassWorldTrans) {
    Node* owner = owner_object();
    Transform* trans = owner->transform();
    btTransform aux; getWorldTransform(aux);

    if(std::abs(aux.getOrigin().getX() - prevPos.getOrigin().getX()) >= 0.1f ||
       std::abs(aux.getOrigin().getY() - prevPos.getOrigin().getY()) >= 0.1f ||
       std::abs(aux.getOrigin().getZ() - prevPos.getOrigin().getZ()) >= 0.1f)
    {
        mMultBody->setWorldTransform(aux);
        prevPos = aux;
        //TODO: incomplete solution
    }
    else
    {
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
        prevPos = physicBody;
    }
    mWorld->markUpdated(this);
}




}
