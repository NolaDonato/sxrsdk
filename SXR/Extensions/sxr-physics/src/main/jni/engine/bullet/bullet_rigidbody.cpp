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

#include "bullet_world.h"
#include "bullet_rigidbody.h"
#include "bullet_sxr_utils.h"
#include "objects/components/sphere_collider.h"
#include "util/sxr_log.h"
#include "../physics_collidable.h"

#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionShapes/btEmptyShape.h>
#include <LinearMath/btDefaultMotionState.h>
#include <LinearMath/btTransform.h>


namespace sxr
{

    BulletRigidBody::BulletRigidBody()
    :   mConstructionInfo(btScalar(0.0f), nullptr, nullptr),
        mRigidBody(nullptr),
        mSimType(SimulationType::DYNAMIC)
    {
        mWorld = nullptr;
        mConstructionInfo.m_linearDamping = 0;
        mConstructionInfo.m_angularDamping = 0;
        mConstructionInfo.m_friction = 0.5f;
        mConstructionInfo.m_rollingFriction = 0;
        mConstructionInfo.m_spinningFriction = 0;
        mConstructionInfo.m_restitution = 0;
        mConstructionInfo.m_linearSleepingThreshold = 0.8f;
        mConstructionInfo.m_angularSleepingThreshold = 1;
        mConstructionInfo.m_additionalDamping = false;
        mConstructionInfo.m_additionalDampingFactor = 0.005f;
        mConstructionInfo.m_additionalLinearDampingThresholdSqr = 0;
        mConstructionInfo. m_additionalAngularDampingThresholdSqr = 0;
        mConstructionInfo.m_additionalAngularDampingFactor = 0.01f;
        mConstructionInfo.m_localInertia = btVector3(0, 0, 0);
    }

    BulletRigidBody::BulletRigidBody(btRigidBody* rigidBody)
    :   mConstructionInfo(rigidBody->isStaticObject() ? 0.f : 1.f / rigidBody->getInvMass(),
                          this, rigidBody->getCollisionShape()),
        mRigidBody(rigidBody),
        mSimType(SimulationType::DYNAMIC)
    {
        mRigidBody->setUserPointer(this);
        mConstructionInfo.m_friction = rigidBody->getFriction();
        mConstructionInfo.m_restitution = rigidBody->getRestitution();
        mConstructionInfo.m_linearDamping = rigidBody->getLinearDamping();
        mConstructionInfo.m_angularDamping = rigidBody->getAngularDamping();
        mConstructionInfo.m_rollingFriction = rigidBody->getRollingFriction();
        mConstructionInfo.m_restitution = rigidBody->getRestitution();
        mConstructionInfo.m_linearSleepingThreshold = rigidBody->getLinearSleepingThreshold();
        mConstructionInfo.m_angularSleepingThreshold = rigidBody->getAngularSleepingThreshold();
        mConstructionInfo.m_additionalDamping = false;
        mConstructionInfo.m_localInertia = rigidBody->getLocalInertia();
        mConstructionInfo.m_additionalDamping = false;
        mConstructionInfo.m_additionalDampingFactor = 0.005f;
        mConstructionInfo.m_additionalLinearDampingThresholdSqr = 0;
        mConstructionInfo. m_additionalAngularDampingThresholdSqr = 0;
        mConstructionInfo.m_additionalAngularDampingFactor = 0.01f;

        if (rigidBody->isStaticObject())
        {
            mSimType = SimulationType::STATIC;
        }
        else if (rigidBody->isKinematicObject())
        {
            mSimType = SimulationType::KINEMATIC;
        }
        mWorld = nullptr;
    }

    BulletRigidBody::~BulletRigidBody()
    {
        finalize();
    }

    void BulletRigidBody::setName(const char* name)
    {
        Node* owner = owner_object();
        if (owner)
        {
            if (!owner->name().empty() &&
                (owner->name() != name))
            {
                mName = name;
                owner->set_name(name);
            }
        }
        else
        {
            mName = name;
        }
    }

    const char* BulletRigidBody::getName() const
    {
        Node* owner = owner_object();
        if (owner && !owner->name().empty())
        {
            const std::string& name =  owner->name();
            if (!name.empty())
            {
                mName = name;
            }
        }
        if (mName.empty())
        {
            return nullptr;
        }
        return mName.c_str();
    }

    void BulletRigidBody::setSimulationType(PhysicsRigidBody::SimulationType type)
    {
        mSimType = type;
        if (mRigidBody)
        {
            sync(SyncOptions::PROPERTIES);
        }
    }

    void BulletRigidBody::onDisable(Node* owner)
    {
        if (mRigidBody)
        {
            if ((mRigidBody->getCollisionFlags() &
                 ~(btCollisionObject::CollisionFlags::CF_KINEMATIC_OBJECT |
                   btCollisionObject::CollisionFlags::CF_STATIC_OBJECT)) == 0)
            {
                mRigidBody->setActivationState(DISABLE_SIMULATION);
            }
        }
    }

    void BulletRigidBody::onEnable(Node* owner)
    {
        if (mRigidBody)
        {
            if ((mRigidBody->getCollisionFlags() &
                 ~(btCollisionObject::CollisionFlags::CF_KINEMATIC_OBJECT |
                   btCollisionObject::CollisionFlags::CF_STATIC_OBJECT)) == 0)
            {
                mRigidBody->setActivationState(ACTIVE_TAG);
            }
        }
    }

    void BulletRigidBody::sync(int options)
    {
        int collisionFlags = 0;
        Node* owner = owner_object();
        Collider* collider = (Collider*) owner->getComponent(COMPONENT_TYPE_COLLIDER);

        if (!collider)
        {
            LOGE("PHYSICS: Cannot attach rigid body without collider");
        }
        if (mRigidBody == nullptr)
        {
            mRigidBody = new btRigidBody(mConstructionInfo);
            mRigidBody->setUserPointer(this);
            options |= SyncOptions::COLLISION_SHAPE | SyncOptions::PROPERTIES;
        }
        if (options & SyncOptions::COLLISION_SHAPE)
        {
            updateCollider(owner, options);
        }
        if (options & SyncOptions::PROPERTIES)
        {
            switch (mSimType)
            {
                case SimulationType::DYNAMIC:
                mRigidBody->setCollisionFlags(collisionFlags &
                                              ~(btCollisionObject::CollisionFlags::CF_KINEMATIC_OBJECT |
                                                btCollisionObject::CollisionFlags::CF_STATIC_OBJECT));
                mRigidBody->setActivationState(enabled() ? ACTIVE_TAG : DISABLE_SIMULATION);
                break;

                case SimulationType::STATIC:
                mRigidBody->setCollisionFlags(
                        (collisionFlags | btCollisionObject::CollisionFlags::CF_STATIC_OBJECT) &
                        ~btCollisionObject::CollisionFlags::CF_KINEMATIC_OBJECT);
                mRigidBody->setActivationState(ISLAND_SLEEPING);
                break;

                case SimulationType::KINEMATIC:
                mRigidBody->setCollisionFlags(
                        (collisionFlags | btCollisionObject::CollisionFlags::CF_KINEMATIC_OBJECT) &
                        ~btCollisionObject::CollisionFlags::CF_STATIC_OBJECT);
                mRigidBody->setActivationState(ISLAND_SLEEPING);
                break;
            }
            mConstructionInfo.m_motionState = this;
            mRigidBody->setMotionState(this);
        }
    }

    void BulletRigidBody::onAddedToWorld(PhysicsWorld* world)
    {
        sync();
        mWorld = static_cast<BulletWorld*>(world);
    }

    void BulletRigidBody::updateCollider(Node* owner, int options)
    {
        btCollisionShape* oldShape = nullptr;
        btCollisionShape* newShape = nullptr;
        Transform* trans = owner->transform();
        Collider* collider = (Collider*) owner->getComponent(COMPONENT_TYPE_COLLIDER);
        btVector3 ownerScale(trans->scale_x(), trans->scale_y(), trans->scale_z());
        bool creating = (mRigidBody == nullptr);

        if (collider == nullptr)
        {
            return;
        }
        if (creating)
        {
            mRigidBody = new btRigidBody(mConstructionInfo);
            mRigidBody->setUserPointer(this);
            if ((mRigidBody->getCollisionFlags() &
                 ~(btCollisionObject::CollisionFlags::CF_KINEMATIC_OBJECT |
                   btCollisionObject::CollisionFlags::CF_STATIC_OBJECT)) == 0)
            {
                mRigidBody->setActivationState(ACTIVE_TAG);
            }
            newShape = convertCollider2CollisionShape(collider);
        }
        else if (options & SyncOptions::COLLISION_SHAPE)
        {
            newShape = convertCollider2CollisionShape(collider);
            oldShape = mRigidBody->getCollisionShape();
            if (oldShape)
            {
                delete oldShape;
            }
        }
        if (newShape)
        {
            mRigidBody->setCollisionShape(newShape);
            newShape->setLocalScaling(ownerScale);
            if (mConstructionInfo.m_mass > 0)
            {
                newShape->calculateLocalInertia(mConstructionInfo.m_mass,
                                                mConstructionInfo.m_localInertia);
            }
        }
    }

    void BulletRigidBody::finalize()
    {
        if (mRigidBody)
        {
            if (mWorld)
            {
                mWorld->getPhysicsWorld()->removeRigidBody(mRigidBody);
            }
            if (mRigidBody->getCollisionShape())
            {
                mConstructionInfo.m_collisionShape = 0;
                delete mRigidBody->getCollisionShape();
            }
            delete mRigidBody;
            mRigidBody = 0;
        }
    }

    void BulletRigidBody::getRotation(float& w, float& x, float& y, float& z)
    {
        Node* owner = owner_object();

        if (owner != nullptr)
        {
            Transform* t = owner->transform();
            x = t->rotation_x();
            y = t->rotation_y();
            z = t->rotation_z();
            w = t->rotation_w();
        }
        else
        {
            x = 0;
            y = 0;
            z = 0;
            w = 1;
        }
    }

    void BulletRigidBody::getTranslation(float& x, float& y, float& z)
    {
        Node* owner = owner_object();

        if (owner != nullptr)
        {
            Transform* t = owner->transform();
            x = t->position_x();
            y = t->position_y();
            z = t->position_z();
        }
        else
        {
            x = 0;
            y = 0;
            z = 0;
        }
    }

    void BulletRigidBody::setCenterOfMass(Transform* t)
    {
        if (mRigidBody != nullptr)
        {
            mRigidBody->setCenterOfMassTransform(convertTransform2btTransform(t));
        }
    }

    void BulletRigidBody::getWorldTransform(btTransform &centerOfMassWorldTrans) const
    {
        Transform* trans = owner_object()->transform();

        centerOfMassWorldTrans = convertTransform2btTransform(trans);
    }

    void BulletRigidBody::setWorldTransform(const btTransform &centerOfMassWorldTrans)
    {
        Node* owner = owner_object();

        if (!owner->enabled())
        {
            return;
        }
        Transform* trans = owner->transform();
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
        LOGD("PHYSICS: rigid body %s pos = %f, %f, %f", owner->name().c_str(),
                trans->position_x(),
                trans->position_y(),
                trans->position_z());
    }

    void BulletRigidBody::applyCentralForce(float x, float y, float z)
    {
        if (mRigidBody != nullptr)
        {
            mRigidBody->applyCentralForce(btVector3(x, y, z));
            if (!mRigidBody->isActive())
            {
                mRigidBody->activate(true);
            }
        }
    }

    void BulletRigidBody::applyForce(float force_x, float force_y, float force_z,
                                     float rel_pos_x, float rel_pos_y, float rel_pos_z)
    {
        if (mRigidBody != nullptr)
        {
            mRigidBody->applyForce(btVector3(force_x, force_y, force_z),
                                   btVector3(rel_pos_x, rel_pos_y, rel_pos_z));
            if (!mRigidBody->isActive())
            {
                mRigidBody->activate(true);
            }
        }
    }

    void BulletRigidBody::applyCentralImpulse(float x, float y, float z)
    {
        if (mRigidBody != nullptr)
        {
            mRigidBody->applyCentralImpulse(btVector3(x, y, z));
            if (!mRigidBody->isActive())
            {
                mRigidBody->activate(true);
            }
        }
    }

    void BulletRigidBody::applyImpulse(float impulse_x, float impulse_y, float impulse_z,
                                       float rel_pos_x, float rel_pos_y, float rel_pos_z)
    {
        if (mRigidBody != nullptr)
        {
            mRigidBody->applyImpulse(btVector3(impulse_x, impulse_y, impulse_z),
                                     btVector3(rel_pos_x, rel_pos_y, rel_pos_z));
            if (!mRigidBody->isActive())
            {
                mRigidBody->activate(true);
            }
        }
    }

    void BulletRigidBody::applyTorque(float x, float y, float z)
    {
        if (mRigidBody != nullptr)
        {
            mRigidBody->applyTorque(btVector3(x, y, z));
            if (!mRigidBody->isActive())
            {
                mRigidBody->activate(true);
            }
        }
    }

    void BulletRigidBody::applyTorqueImpulse(float x, float y, float z)
    {
        if (mRigidBody != nullptr)
        {
            mRigidBody->applyTorqueImpulse(btVector3(x, y, z));
            if (!mRigidBody->isActive())
            {
                mRigidBody->activate(true);
            }
        }
    }


    void BulletRigidBody::setGravity(float x, float y, float z)
    {
        if (mRigidBody != nullptr)
        {
            mRigidBody->setGravity(btVector3(x, y, z));
        }
    }

    void BulletRigidBody::setDamping(float linear, float angular)
    {
        mConstructionInfo.m_linearDamping = linear;
        mConstructionInfo.m_angularDamping = angular;
        if (mRigidBody != nullptr)
        {
            mRigidBody->setDamping(linear, angular);
        }
    }

    void BulletRigidBody::setLinearVelocity(float x, float y, float z)
    {
        if (mRigidBody != nullptr)
        {
            mRigidBody->setLinearVelocity(btVector3(x, y, z));
        }
    }

    void BulletRigidBody::setAngularVelocity(float x, float y, float z)
    {
        if (mRigidBody != nullptr)
        {
            mRigidBody->setAngularVelocity(btVector3(x, y, z));
        }
    }

    void BulletRigidBody::setAngularFactor(float x, float y, float z)
    {
        if (mRigidBody != nullptr)
        {
            mRigidBody->setAngularFactor(btVector3(x, y, z));
        }
    }

    void BulletRigidBody::setLinearFactor(float x, float y, float z)
    {
        if (mRigidBody != nullptr)
        {
            mRigidBody->setLinearFactor(btVector3(x, y, z));
        }
    }

    void BulletRigidBody::setFriction(float n)
    {
        mConstructionInfo.m_friction = n;
        if (mRigidBody != nullptr)
        {
            mRigidBody->setFriction(n);
        }
    }

    void BulletRigidBody::setRestitution(float n)
    {
        mConstructionInfo.m_restitution = n;
        if (mRigidBody != nullptr)
        {
            mRigidBody->setRestitution(n);
        }
    }

    void BulletRigidBody::setSleepingThresholds(float linear, float angular)
    {
        if (mRigidBody != nullptr)
        {
            mRigidBody->setSleepingThresholds(linear, angular);
        }
    }

    void BulletRigidBody::setCcdMotionThreshold(float n)
    {
        if (mRigidBody != nullptr)
        {
            mRigidBody->setCcdMotionThreshold(n);
        }
    }

    void BulletRigidBody::setCcdSweptSphereRadius(float n)
    {
        if (mRigidBody != nullptr)
        {
            mRigidBody->setCcdSweptSphereRadius(n);
        }
    }

    void BulletRigidBody::setContactProcessingThreshold(float n)
    {
        if (mRigidBody != nullptr)
        {
            mRigidBody->setContactProcessingThreshold(n);
        }
    }

    void BulletRigidBody::setIgnoreCollisionCheck(PhysicsRigidBody *collisionObj, bool ignore)
    {
        if (mRigidBody != nullptr)
        {
            mRigidBody->setIgnoreCollisionCheck(((BulletRigidBody*) collisionObj)->getRigidBody(), ignore);
        }
    }

    void BulletRigidBody::getGravity(float *v3) const
    {
        if (mRigidBody != nullptr)
        {
            btVector3 result = mRigidBody->getGravity();
            v3[0] = result.getX();
            v3[1] = result.getY();
            v3[2] = result.getZ();
        }
        else
        {
            v3[0] = 0;
            v3[1] = 0;
            v3[2] = 0;
        }
    }

    void BulletRigidBody::getDamping(float &angular, float &linear) const
    {
        linear = mConstructionInfo.m_linearDamping;
        angular = mConstructionInfo.m_angularDamping;
    }

    void BulletRigidBody::getLinearVelocity(float *v3) const
    {
        if (mRigidBody != nullptr)
        {
            btVector3 result = mRigidBody->getLinearVelocity();
            v3[0] = result.getX();
            v3[1] = result.getY();
            v3[2] = result.getZ();
        }
        else
        {
            v3[0] = 0;
            v3[1] = 0;
            v3[2] = 0;
        }
    }

    void BulletRigidBody::getAngularVelocity(float *v3) const
    {
        if (mRigidBody != nullptr)
        {
            btVector3 result = mRigidBody->getAngularVelocity();
            v3[0] = result.getX();
            v3[1] = result.getY();
            v3[2] = result.getZ();
        }
        else
        {
            v3[0] = 0;
            v3[1] = 0;
            v3[2] = 0;
        }
    }

    void BulletRigidBody::getAngularFactor(float *v3) const
    {
        if (mRigidBody != nullptr)
        {
            btVector3 result = mRigidBody->getAngularFactor();
            v3[0] = result.getX();
            v3[1] = result.getY();
            v3[2] = result.getZ();
        }
        else
        {
            v3[0] = 1;
            v3[1] = 1;
            v3[2] = 1;
        }
    }

    void BulletRigidBody::getLinearFactor(float *v3) const
    {
        if (mRigidBody != nullptr)
        {
            btVector3 result = mRigidBody->getLinearFactor();
            v3[0] = result.getX();
            v3[1] = result.getY();
            v3[2] = result.getZ();
        }
        else
        {
            v3[0] = 1;
            v3[1] = 1;
            v3[2] = 1;
        }
    }

    float  BulletRigidBody::getCcdMotionThreshold() const
    {
        if (mRigidBody != nullptr)
        {
            return mRigidBody->getCcdMotionThreshold();
        }
        return 0;
    }

    float  BulletRigidBody::getCcdSweptSphereRadius() const
    {
        if (mRigidBody != nullptr)
        {
            return mRigidBody->getCcdSweptSphereRadius();
        }
        return 0;
    }

    float  BulletRigidBody::getContactProcessingThreshold() const
    {
        if (mRigidBody != nullptr)
        {
            return mRigidBody->getContactProcessingThreshold();
        }
        return BT_LARGE_FLOAT;
    }

}
