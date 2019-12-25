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
#include "../../bullet3/include/BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"

namespace sxr {

    BulletJoint::BulletJoint(float mass, int numJoints, int collisionGroup)
    : PhysicsJoint(mass, numJoints),
      mMultiBody(nullptr),
      mCollider(nullptr),
      mParent(nullptr),
      mJointIndex(-1),
      mAxis(1, 0, 0),
      mPivot(0, 0, 0),
      mWorld(nullptr),
      mMass(mass),
      mFriction(0),
      mScale(1, 1, 1),
      mNeedsSync(SyncOptions::ALL),
      mLinearDamping(0),
      mAngularDamping(0),
      mLocalInertia(0, 0, 0),
      mJointType(JointType::sphericalJoint)
    {
        mCollisionGroup = collisionGroup;
        if (mass != 0)
        {
            mSimType = DYNAMIC;
            mCollisionMask = btBroadphaseProxy::AllFilter;
        }
        else
        {
            mSimType = STATIC;
            mCollisionMask = btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter;
        }
    }

    BulletJoint::BulletJoint(BulletJoint* parent, JointType jointType, int jointIndex, float mass, int collisionGroup)
    : PhysicsJoint(parent, jointType, jointIndex, mass),
      mMultiBody(nullptr),
      mParent(parent),
      mCollider(nullptr),
      mJointIndex(jointIndex - 1),
      mLocalInertia(0, 0, 0),
      mAxis(1, 0, 0),
      mScale(1, 1, 1),
      mMass(mass),
      mFriction(0),
      mNeedsSync(SyncOptions::ALL),
      mPivot(0, 0, 0),
      mJointType(jointType),
      mLinearDamping(0),
      mAngularDamping(0),
      mWorld(nullptr)
    {
        mCollisionGroup = collisionGroup;
        if (mass != 0)
        {
            mSimType = DYNAMIC;
            mCollisionMask = btBroadphaseProxy::AllFilter;
        }
        else
        {
            mSimType = STATIC;
            mCollisionMask = btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter;
        }
    }

    BulletJoint::BulletJoint(BulletJoint* parent, int jointIndex, int collisionGroup)
    :   PhysicsJoint(parent, (JointType) parent->getMultiBody()->getLink(jointIndex).m_jointType, jointIndex, 0),
        mParent(parent),
        mNeedsSync(SyncOptions::IMPORTED),
        mMultiBody(parent->getMultiBody()),
        mJointIndex(jointIndex),
        mFriction(0),
        mScale(1, 1, 1),
        mWorld(nullptr)
    {
        btMultibodyLink& link = mMultiBody->getLink(jointIndex);
        link.m_userPtr = this;
        mCollider = link.m_collider;
        mJointType = (JointType) link.m_jointType;
        mMass = link.m_mass;
        mLinearDamping = mMultiBody->getLinearDamping();
        mAngularDamping = mMultiBody->getAngularDamping();
        mLocalInertia = glm::vec3(link.m_inertiaLocal.x(), link.m_inertiaLocal.y(), link.m_inertiaLocal.z());
        mCollisionGroup = collisionGroup;
        if (mMass != 0)
        {
            mSimType = DYNAMIC;
            mCollisionMask = btBroadphaseProxy::AllFilter;
        }
        else
        {
            mSimType = STATIC;
            mCollisionMask = btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter;
        }
        if (mCollider)
        {
            btCollisionShape* shape = mCollider->getCollisionShape();
            if (shape)
            {
                btVector3 v = shape->getLocalScaling();
                mScale.x = v.x();
                mScale.y = v.y();
                mScale.z = v.z();
            }
            mCollider->setUserPointer(this);
        }
    }

    void BulletJoint::setName(const char* name)
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

    int BulletJoint::addJointToBody(PhysicsJoint* child)
    {
        BulletRootJoint* newRoot = findRoot();

        if (newRoot)
        {
            return newRoot->addJointToBody(child);
        }
        return -1;
    }

    void BulletJoint::removeJointFromBody(int jointIndex)
    {
        BulletRootJoint* root = findRoot();

        if (root)
        {
            root->removeJointFromBody(jointIndex);
        }
    }

    const char* BulletJoint::getName() const
    {
        Node* owner = owner_object();
        if (owner && !owner->name().empty())
        {
            const std::string& name = owner->name();
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

    const BulletRootJoint* BulletJoint::findRoot() const
    {
        if (mMultiBody)
        {
            return (BulletRootJoint*) mMultiBody->getUserPointer();
        }
        if (mParent)
        {
            return mParent->findRoot();
        }
        return nullptr;
    }

    BulletRootJoint* BulletJoint::findRoot()
    {
        if (mMultiBody)
        {
            return (BulletRootJoint*) mMultiBody->getUserPointer();
        }
        if (mParent)
        {
            return mParent->findRoot();
        }
        return nullptr;
    }

    void BulletJoint::setLinearDamping(float ld)
    {
        mLinearDamping = ld;
    }

    void BulletJoint::setAngularDamping(float ad)
    {
        mAngularDamping = ad;
    }


    void BulletJoint::setMass(float mass)
    {
        if (mMass != mass)
        {
            LOGV("BULLET: joint %s mass %3f to %3f",
                 getName(), mMass, mass);
            mMass = mass;
            mNeedsSync |= SyncOptions::PROPERTIES;
        }
    }

    void BulletJoint::setFriction(float friction)
    {
        mFriction = friction;
        mNeedsSync |= SyncOptions::PROPERTIES;
    }

    void BulletJoint::setPivot(const glm::vec3& pivot)
    {
        if (pivot != mPivot)
        {
            LOGV("BULLET: joint %s pivot (%3f, %3f, %3f) to  (%3f, %3f, %3f)",
                 getName(), mPivot.x, mPivot.y, mPivot.z, pivot.x, pivot.y, pivot.z);
            mPivot = pivot;
            mNeedsSync |= SyncOptions::PROPERTIES;
        }
    }

    void BulletJoint::setAxis(const glm::vec3& axis)
    {
        if (axis != mAxis)
        {
            LOGV("BULLET: joint %s axis (%3f, %3f, %3f) to  (%3f, %3f, %3f)",
                 getName(), mAxis.x, mAxis.y, mAxis.z, axis.x, axis.y, axis.z);
            mAxis = axis;
            mNeedsSync |= SyncOptions::PROPERTIES;
        }
    }

    void BulletJoint::setMaxAppliedImpulse(float v)
    {
        // this should never be called, only for root joint
    }

    void BulletJoint::setMaxCoordVelocity(float v)
    {
        // this should never be called, only for root joint
    }

    Skeleton* BulletJoint::getSkeleton() const
    {
        const BulletRootJoint* root = findRoot();
        return root ? root->getSkeleton() : nullptr;
    }

    void BulletJoint::update(int jointIndex, BulletJoint* parent)
    {
        btMultiBody* oldMB = mMultiBody;
        if (parent == nullptr)
        {
            if (oldMB)
            {
                btMultibodyLink& link = oldMB->getLink(jointIndex);
                link.m_collider = nullptr;
                link.m_userPtr = nullptr;
            }
            mParent = nullptr;
            mMultiBody = nullptr;
            return;
        }
        mMultiBody = parent->getMultiBody();
        mParent = parent;
        mJointIndex = jointIndex;
        if (mCollider)
        {
            mCollider->setUserPointer(this);
            if (mMultiBody)
            {
                btMultibodyLink& link = mMultiBody->getLink(jointIndex);

                mCollider->m_link = jointIndex;
                mCollider->m_multiBody = mMultiBody;
                mCollider->setUserPointer(this);
                link.m_userPtr = this;
                link.m_collider = mCollider;
                link.m_parent = parent->getJointIndex();
            }
            LOGV("BULLET: updating link collider %s", getName());
        }
    }

    int BulletJoint::getNumJoints() const { return findRoot()->getNumJoints(); }

    void BulletJoint::getWorldTransform(btTransform& t)
    {
        Node* owner = owner_object();
        Transform* trans = owner->transform();
        t = convertTransform2btTransform(trans);
    }

    void BulletJoint::setPhysicsTransform()
    {
        if (mCollider)
        {
            Node* owner = owner_object();
            Transform* trans = owner->transform();
            btTransform t = convertTransform2btTransform(trans);
            btVector3 pos = t.getOrigin();

//            LOGD("BULLET: UPDATE %s, %f, %f, %f", owner->name().c_str(), pos.getX(), pos.getY(), pos.getZ());
            mCollider->setWorldTransform(t);
        }
    }

    void BulletJoint::getPhysicsTransform()
    {
        if (mCollider == nullptr)
        {
            return;
        }
        Node* owner = owner_object();
        btTransform t = mCollider->getWorldTransform();
        btVector3 pos = t.getOrigin();
        btQuaternion rot = t.getRotation();
        Node* parent = owner->parent();
        Transform* trans = owner->transform();

        if ((parent != nullptr) && (parent->parent() != nullptr))
        {
            float matrixData[16];

            t.getOpenGLMatrix(matrixData);
            glm::mat4 worldMatrix(glm::make_mat4(matrixData));
            glm::mat4 parentWorld(parent->transform()->getModelMatrix(false));
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
        mNeedsSync = 0;
//        LOGD("BULLET: JOINT %s %3f, %3f, %3f",owner->name().c_str(),pos.x(),pos.y(),pos.z());
    }

    void BulletJoint::applyCentralForce(float x, float y, float z)
    {
        btVector3 force(x, y, z);
        if (mMultiBody)
        {
            mMultiBody->addLinkForce(getJointIndex(), force);
        }
    }

    void BulletJoint::applyTorque(float x, float y, float z)
    {
        if (mMultiBody)
        {
            btVector3 torque(x, y, z);
            mMultiBody->addJointTorqueMultiDof(mJointIndex, torque);
            mMultiBody->addLinkTorque(mJointIndex, torque);
        }
    }

    void BulletJoint::applyTorque(float t)
    {
        if (mMultiBody)
        {
            btVector3 torque(t, 0, 0);
            mMultiBody->addJointTorqueMultiDof(mJointIndex, torque);
            mMultiBody->addLinkTorque(mJointIndex, torque);
        }
    }

    void BulletJoint::setScale(const glm::vec3& s)
    {
        if (s != mScale)
        {
            LOGV("BULLET: joint %s scale (%3f, %3f, %3f) to (%3f, %3f, %3f)",
                 getName(), mScale.x, mScale.y, mScale.z, s.x, s.y, s.z);
            mScale = s;
            mNeedsSync |= SyncOptions::PROPERTIES;
        }
    }

    void BulletJoint::setSimulationType(PhysicsCollidable::SimulationType type)
    {
        if (type != mSimType)
        {
            mSimType = type;
            mNeedsSync |= SyncOptions::PROPERTIES;
            if (mMultiBody)
            {
                sync();
            }
        }
    }

    void BulletJoint::sync(int options)
    {
        BulletJoint* parent = static_cast<BulletJoint*>(getParent());
        bool creating = (mMultiBody == nullptr);
        mMultiBody = parent->getMultiBody();
        btMultibodyLink& link = mMultiBody->getLink(mJointIndex);

        options |= mNeedsSync;
        mNeedsSync = 0;
        updateCollider(owner_object(), options);
        if (options & SyncOptions::TRANSFORM)
        {
            setPhysicsTransform();
        }
        if (creating)
        {
            link.m_parent = parent->getJointIndex();
            link.m_mass = mMass;
            link.m_jointFriction = mFriction;
            link.m_userPtr = this;
            link.m_collider = mCollider;
            link.m_inertiaLocal = btVector3(mLocalInertia.x, mLocalInertia.y, mLocalInertia.z);
            switch (mJointType)
            {
                case JointType::fixedJoint: setupFixed(); break;
                case JointType::prismaticJoint: setupSlider(); break;
                case JointType::revoluteJoint: setupHinge(); break;
                default: setupSpherical(); break;
            }
        }
        else if (options & SyncOptions::PROPERTIES)
        {
            link.m_mass = mMass;
            link.m_jointFriction = mFriction;
            link.m_collider = mCollider;
            link.m_inertiaLocal = btVector3(mLocalInertia.x, mLocalInertia.y, mLocalInertia.z);
            switch (mJointType)
            {
                case JointType::fixedJoint: updateFixed(); break;
                case JointType::prismaticJoint: updateSlider(); break;
                case JointType::revoluteJoint: updateHinge(); break;
                case JointType::sphericalJoint: updateSpherical(); break;
                default: break;
            }
        }
    }

    void BulletJoint::updateCollider(Node* owner, int options)
    {
        btCollisionShape* curShape = nullptr;
        btCollisionShape* newShape = nullptr;
        Collider* collider = (Collider*) owner->getComponent(COMPONENT_TYPE_COLLIDER);

        if (collider == nullptr)
        {
            return;
        }
        if (mCollider == nullptr)
        {
            mCollider = new btMultiBodyLinkCollider(mMultiBody, mJointIndex);
            LOGV("BULLET: creating link collider %s", getName());
            curShape = newShape = convertCollider2CollisionShape(collider);
            mCollider->setCollisionShape(newShape);
            mCollider->m_link = getJointIndex();
            mCollider->setUserPointer(this);
            options |= SyncOptions::PROPERTIES;
        }
        else
        {
            curShape = mCollider->getCollisionShape();
            if ((options & SyncOptions::COLLISION_SHAPE) != 0)
            {
                newShape = convertCollider2CollisionShape(collider);
                options |= SyncOptions::PROPERTIES;
                if (mWorld)
                {
                    btDynamicsWorld *bw = mWorld->getPhysicsWorld();
                    bw->removeCollisionObject(mCollider);
                    mCollider->setCollisionShape(newShape);
                    bw->addCollisionObject(mCollider, mCollisionGroup, mCollisionMask);
                }
                else
                {
                    mCollider->setCollisionShape(newShape);
                }
                if (curShape)
                {
                    delete curShape;
                    curShape = newShape;
                }
            }
        }
        if (options & SyncOptions::PROPERTIES)
        {
            btVector3 localInertia;
            btVector3 scale(mScale.x, mScale.y, mScale.z);

            curShape->setLocalScaling(scale);
            curShape->calculateLocalInertia(getMass(), localInertia);
            mLocalInertia.x = localInertia.x();
            mLocalInertia.y = localInertia.y();
            mLocalInertia.z = localInertia.z();
            int collisionFlags = mCollider->getCollisionFlags();
            switch (mSimType)
            {
                case SimulationType::DYNAMIC:
                mCollisionGroup &= ~(btBroadphaseProxy::StaticFilter | btBroadphaseProxy::KinematicFilter);
                mCollisionMask = btBroadphaseProxy::AllFilter;
                mCollider->setCollisionFlags(collisionFlags &
                                              ~(btCollisionObject::CollisionFlags::CF_KINEMATIC_OBJECT |
                                                btCollisionObject::CollisionFlags::CF_STATIC_OBJECT));
                mCollider->setActivationState(enabled() ? ACTIVE_TAG : WANTS_DEACTIVATION);
                break;

                case SimulationType::STATIC:
                mCollisionGroup |= btBroadphaseProxy::StaticFilter;
                mCollisionGroup &= ~(btBroadphaseProxy::KinematicFilter |
                                     btBroadphaseProxy::DefaultFilter);
                mCollisionMask = btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter;
                mCollider->setCollisionFlags(
                        (collisionFlags | btCollisionObject::CollisionFlags::CF_STATIC_OBJECT) &
                        ~btCollisionObject::CollisionFlags::CF_KINEMATIC_OBJECT);
                mCollider->setActivationState(enabled() ? ISLAND_SLEEPING : WANTS_DEACTIVATION);
                break;

                case SimulationType::KINEMATIC:
                mCollisionGroup |= btBroadphaseProxy::KinematicFilter;
                mCollisionGroup &= ~(btBroadphaseProxy::StaticFilter | btBroadphaseProxy::DefaultFilter);
                mCollisionMask = btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::KinematicFilter;
                mCollider->setCollisionFlags((collisionFlags |
                                               btCollisionObject::CollisionFlags::CF_KINEMATIC_OBJECT) &
                                              ~btCollisionObject::CollisionFlags::CF_STATIC_OBJECT);
                if (enabled())
                {
                    mCollider->setActivationState(DISABLE_DEACTIVATION);
                }
                else
                {
                    mCollider->forceActivationState(WANTS_DEACTIVATION);
                }
                break;
            }
        }
    }

    void BulletJoint::setCollisionMask(int collidesWith)
    {
        mCollisionMask = collidesWith;
        if (mCollider)
        {
            mCollider->getBroadphaseHandle()->m_collisionFilterGroup = mCollisionGroup;
            mCollider->getBroadphaseHandle()->m_collisionFilterMask = collidesWith;
        }
    }

    void BulletJoint::detachFromWorld(bool deleteCollider)
    {
        if (mCollider)
        {
            if (mWorld)
            {
                btMultiBodyDynamicsWorld* w = dynamic_cast<btMultiBodyDynamicsWorld*>(mWorld->getPhysicsWorld());
                w->removeCollisionObject(mCollider);
                mWorld = nullptr;
                LOGD("BULLET: detaching joint %s from world", getName());
            }
            if (deleteCollider)
            {
                btCollisionShape* shape = mCollider->getCollisionShape();
                mMultiBody->getLink(mJointIndex).m_collider = nullptr;
                delete mCollider;
                delete shape;
                mCollider = nullptr;
            }
        }
    }

    void BulletJoint::attachToWorld(PhysicsWorld* w)
    {
        Node* owner = owner_object();
        const char* name = owner->name().c_str();
        BulletWorld* bw = static_cast<BulletWorld*>(w);
        btMultibodyLink* link = getLink();

        if (name)
        {
            mName = name;
        }
        link->m_linkName = mName.c_str();
        link->m_jointName = mName.c_str();
        mWorld = bw;
        bw->getPhysicsWorld()->addCollisionObject(mCollider, mCollisionGroup, mCollisionMask);
        LOGD("BULLET: attaching joint %s to world", getName());
    }

    void BulletJoint::setupFixed()
    {
        BulletJoint*       jointA = static_cast<BulletJoint*>(getParent());
        btVector3          pivotB(mPivot.x, mPivot.y, mPivot.z);
        btTransform        worldA; jointA->getWorldTransform(worldA);
        btTransform        worldB; getWorldTransform(worldB);
        btQuaternion       rotA(worldA.getRotation());
        btVector3          bodyACOM(worldA.getOrigin());
        btVector3          bodyBCOM(worldB.getOrigin());
        btVector3          diffCOM = bodyBCOM + pivotB - bodyACOM;
        btMultibodyLink&   link = mMultiBody->getLink(getJointIndex());

        mMultiBody->setupFixed(getJointIndex(),
                               link.m_mass,
                               link.m_inertiaLocal,
                               jointA->getJointIndex(),
                               rotA,
                               diffCOM,
                               -pivotB,
                               true);
    }

    void BulletJoint::updateFixed()
    {
        BulletJoint*       jointA = static_cast<BulletJoint*>(getParent());
        btVector3          pivotB(mPivot.x, mPivot.y, mPivot.z);
        btTransform        worldA;  jointA->getWorldTransform(worldA);
        btTransform        worldB; getWorldTransform(worldB);
        btQuaternion       rotA(worldA.getRotation());
        btVector3          bodyACOM(worldA.getOrigin());
        btVector3          bodyBCOM(worldB.getOrigin());
        btVector3          diffCOM = bodyBCOM + pivotB - bodyACOM;
        btMultibodyLink&   link = mMultiBody->getLink(getJointIndex());

        link.m_zeroRotParentToThis = rotA;
        link.m_eVector = diffCOM;
        link.m_dVector = -pivotB;
        link.updateCacheMultiDof();
    }

    void BulletJoint::setupSpherical()
    {
        BulletJoint*       jointA = static_cast<BulletJoint*>(getParent());
        btVector3          pivotB(mPivot.x, mPivot.y, mPivot.z);
        btTransform        worldA;  jointA->getWorldTransform(worldA);
        btTransform        worldB; getWorldTransform(worldB);
        btQuaternion       rotA(worldA.getRotation());
        btVector3          bodyACOM(worldA.getOrigin());
        btVector3          bodyBCOM(worldB.getOrigin());
        btVector3          diffCOM = bodyBCOM + pivotB - bodyACOM;
        btMultibodyLink&   link = mMultiBody->getLink(getJointIndex());

        mMultiBody->setupSpherical(getJointIndex(),
                                   link.m_mass,
                                   link.m_inertiaLocal,
                                   jointA->getJointIndex(),
                                   rotA,
                                   diffCOM,
                                   -pivotB, true);
    }

    void BulletJoint::updateSpherical()
    {
        BulletJoint*       jointA = static_cast<BulletJoint*>(getParent());
        btVector3          pivotB(mPivot.x, mPivot.y, mPivot.z);
        btTransform        worldA; jointA->getWorldTransform(worldA);
        btTransform        worldB; getWorldTransform(worldB);
        btQuaternion       rotA(worldA.getRotation());
        btVector3          bodyACOM(worldA.getOrigin());
        btVector3          bodyBCOM(worldB.getOrigin());
        btVector3          diffCOM = bodyBCOM + pivotB - bodyACOM;
        btMultibodyLink&   link = mMultiBody->getLink(getJointIndex());

        link.m_zeroRotParentToThis = rotA;
        link.m_eVector = diffCOM;
        link.m_dVector = -pivotB;
        link.setAxisBottom(0, link.getAxisTop(0).cross(link.m_dVector));
        link.setAxisBottom(1, link.getAxisTop(1).cross(link.m_dVector));
        link.setAxisBottom(2, link.getAxisTop(2).cross(link.m_dVector));
        link.updateCacheMultiDof();
    }

    /***
     * The hinge joint is set up by choosing a hinge axis in the hinge coordinate system.
     * Below we choose the X axis. To map the SXR world coordinate system into the
     * hinge coordinate system we define a rotation frame with the hinge axis as X,
     * the vector between bodyB and its pivot as Y (up axis) and the remaining
     * axis is the cross between the two (normal to the plane defined by hinge
     * and pivot axes). This rotation (in quaternion form) is the rotParentToThis
     * argument to setupRevolute.
     *
     * The vector from bodyB's center to bodyB's pivot is supplied as the
     * bodyB pivot from the constraint (getPivot()). This vector is the
     * value for thisPivotToThisComOffset in setupRevolute.
     *
     * The parentComToThisPivotOffset argument is the difference between
     * bodyB center and bodyA center plus the bodyB pivot (the vector
     * from bodyA center to bodyB's pivot).
      */
    void BulletJoint::setupHinge()
    {
        BulletJoint*        jointA = static_cast<BulletJoint*>(getParent());
        btVector3           pivotB(mPivot.x, mPivot.y, mPivot.z);
        btTransform         worldA; jointA->getWorldTransform(worldA);
        btTransform         worldB; getWorldTransform(worldB);
        btQuaternion        rotA(worldA.getRotation());
        btVector3           bodyACOM(worldA.getOrigin());
        btVector3           bodyBCOM(worldB.getOrigin());
        btVector3           diffCOM = bodyBCOM + pivotB - bodyACOM;
        btVector3           hingeAxis(mAxis.x, mAxis.y, mAxis.z);
        btMultibodyLink&    link = mMultiBody->getLink(getJointIndex());

        mMultiBody->setupRevolute(getJointIndex(),
                          link.m_mass,
                          link.m_inertiaLocal,
                          jointA->getJointIndex(),
                          rotA,
                          hingeAxis.normalized(),
                          diffCOM,
                          -pivotB, false);
    }

    void BulletJoint::updateHinge()
    {
        BulletJoint*        jointA = static_cast<BulletJoint*>(getParent());
        btVector3           pivotB(mPivot.x, mPivot.y, mPivot.z);
        btTransform         worldA; jointA->getWorldTransform(worldA);
        btTransform         worldB; getWorldTransform(worldB);
        btQuaternion        rotA(worldA.getRotation());
        btVector3           bodyACOM(worldA.getOrigin());
        btVector3           bodyBCOM(worldB.getOrigin());
        btVector3           diffCOM = bodyBCOM + pivotB - bodyACOM;
        btVector3           hingeAxis(mAxis.x, mAxis.y, mAxis.z);
        btMultibodyLink&    link = mMultiBody->getLink(getJointIndex());

        hingeAxis.normalize();
        link.m_zeroRotParentToThis = rotA;
        link.m_eVector = diffCOM;
        link.m_dVector = -pivotB;
        link.setAxisTop(0, hingeAxis);
        link.setAxisBottom(0, hingeAxis.cross(link.m_dVector));
        link.updateCacheMultiDof();
    }

    void BulletJoint::setupSlider()
    {
        BulletJoint*        jointA = static_cast<BulletJoint*>(getParent());
        btVector3           pivotB(mPivot.x, mPivot.y, mPivot.z);
        btTransform         worldA; jointA->getWorldTransform(worldA);
        btTransform         worldB; getWorldTransform(worldB);
        btQuaternion        rotA(worldA.getRotation());
        btVector3           bodyACOM(worldA.getOrigin());
        btVector3           bodyBCOM(worldB.getOrigin());
        btVector3           diffCOM = bodyBCOM + pivotB - bodyACOM;
        btVector3           sliderAxis(bodyBCOM - bodyACOM);
        btMultibodyLink&    link = mMultiBody->getLink(getJointIndex());

        mMultiBody->setupPrismatic(getJointIndex(),
                                  link.m_mass,
                                  link.m_inertiaLocal,
                                  jointA->getJointIndex(),
                                  rotA,
                                  sliderAxis.normalized(),
                                  diffCOM,
                                  -pivotB,
                                  false);
    }

    void BulletJoint::updateSlider()
    {
        BulletJoint*        jointA = static_cast<BulletJoint*>(getParent());
        btVector3           pivotB(mPivot.x, mPivot.y, mPivot.z);
        btTransform         worldA; jointA->getWorldTransform(worldA);
        btTransform         worldB; getWorldTransform(worldB);
        btQuaternion        rotA(worldA.getRotation());
        btVector3           bodyACOM(worldA.getOrigin());
        btVector3           bodyBCOM(worldB.getOrigin());
        btVector3           diffCOM = bodyBCOM + pivotB - bodyACOM;
        btVector3           sliderAxis(bodyBCOM - bodyACOM);
        btMultibodyLink&    link = mMultiBody->getLink(getJointIndex());

        sliderAxis.normalize();
        link.m_zeroRotParentToThis = rotA;
        link.m_eVector = diffCOM;
        link.m_dVector = -pivotB;
        link.setAxisBottom(0, sliderAxis);
        link.updateCacheMultiDof();
    }

    void BulletJoint::onDisable(Node* owner)
    {
        if (mCollider)
        {
            mCollider->setActivationState(WANTS_DEACTIVATION);
        }
    }

    void BulletJoint::onEnable(Node* owner)
    {
        if (mCollider)
        {
            if (mSimType == SimulationType::DYNAMIC)
            {
                mCollider->activate(ACTIVE_TAG);
            }
            else
            {
                mCollider->setActivationState(ISLAND_SLEEPING);
            }
        }
    }
}
