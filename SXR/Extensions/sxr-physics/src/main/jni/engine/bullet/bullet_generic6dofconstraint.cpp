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

//
// Created by c.bozzetto on 09/06/2017.
//

#include "bullet_generic6dofconstraint.h"
#include "bullet_joint.h"
#include "bullet_rigidbody.h"
#include "bullet_sxr_utils.h"

#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h>
#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#include "glm/gtc/type_ptr.hpp"

static const char tag[] = "PHYSICS";

namespace sxr {

    BulletGeneric6dofConstraint::BulletGeneric6dofConstraint(
            PhysicsRigidBody* rigidBodyA, float const joint[], float const rotationA[],
            float const rotationB[]) {
        mGeneric6DofConstraint = 0;

        mRigidBodyA = reinterpret_cast<BulletRigidBody*>(rigidBodyA);

        mBreakingImpulse = SIMD_INFINITY;
        mPosition.x = joint[0];
        mPosition.y = joint[1];
        mPosition.z = joint[2];
        mRotationA = glm::make_mat3(rotationA);
        mRotationB = glm::make_mat3(rotationB);
    }

    BulletGeneric6dofConstraint::BulletGeneric6dofConstraint(btGeneric6DofConstraint *constraint)
    {
        mGeneric6DofConstraint = constraint;
        mRigidBodyA = static_cast<BulletRigidBody*>(constraint->getRigidBodyA().getUserPointer());
        constraint->setUserConstraintPtr(this);
    }

    BulletGeneric6dofConstraint::~BulletGeneric6dofConstraint()
    {
        if (mGeneric6DofConstraint)
        {
            delete mGeneric6DofConstraint;
        }
    }

    void BulletGeneric6dofConstraint::setLinearLowerLimits(float limitX, float limitY, float limitZ)
    {
        if (mGeneric6DofConstraint)
        {
            mGeneric6DofConstraint->setLinearLowerLimit(btVector3(limitX, limitY, limitZ));
        }
        mLinearLowerLimits = glm::vec3(limitX, limitY, limitZ);
    }

    const glm::vec3& BulletGeneric6dofConstraint::getLinearLowerLimits() const
    {
        if (mGeneric6DofConstraint)
        {
            btVector3 t;
            mGeneric6DofConstraint->getLinearLowerLimit(t);
            mLinearLowerLimits.x = t.x();
            mLinearLowerLimits.y = t.y();
            mLinearLowerLimits.z = t.z();
        }
        return mLinearLowerLimits;
    }

    void BulletGeneric6dofConstraint::setLinearUpperLimits(float limitX, float limitY, float limitZ)
    {
        if (mGeneric6DofConstraint)
        {
            mGeneric6DofConstraint->setLinearUpperLimit(btVector3(limitX, limitY, limitZ));
        }
        mLinearUpperLimits = glm::vec3(limitX, limitY, limitZ);
    }

    const glm::vec3& BulletGeneric6dofConstraint::getLinearUpperLimits() const
    {
        if (mGeneric6DofConstraint)
        {
            btVector3 t;
            mGeneric6DofConstraint->getLinearUpperLimit(t);
            mLinearUpperLimits.x = t.x();
            mLinearUpperLimits.y = t.y();
            mLinearUpperLimits.z = t.z();
        }
        return mLinearUpperLimits;
    }

    void BulletGeneric6dofConstraint::setAngularLowerLimits(float limitX, float limitY, float limitZ)
    {
        if (mGeneric6DofConstraint)
        {
            mGeneric6DofConstraint->setAngularLowerLimit(btVector3(limitX, limitY, limitZ));
        }
        mAngularLowerLimits = glm::vec3(limitX, limitY, limitZ);
    }

    const glm::vec3&  BulletGeneric6dofConstraint::getAngularLowerLimits() const
    {
        if (mGeneric6DofConstraint)
        {
            btVector3 t;
            mGeneric6DofConstraint->getAngularLowerLimit(t);
            mAngularLowerLimits.x = t.x();
            mAngularLowerLimits.y = t.y();
            mAngularLowerLimits.z = t.z();
        }
        return mAngularLowerLimits;
    }

    void BulletGeneric6dofConstraint::setAngularUpperLimits(float limitX, float limitY, float limitZ)
    {
        if ( mGeneric6DofConstraint)
        {
            mGeneric6DofConstraint->setAngularUpperLimit(btVector3(limitX, limitY, limitZ));
        }
        mAngularUpperLimits = glm::vec3(limitX, limitY, limitZ);
    }

    const glm::vec3& BulletGeneric6dofConstraint::getAngularUpperLimits() const
    {
        if (mGeneric6DofConstraint)
        {
            btVector3 t;
            mGeneric6DofConstraint->getAngularUpperLimit(t);
            mAngularUpperLimits.x = t.x();
            mAngularUpperLimits.y = t.y();
            mAngularUpperLimits.z = t.z();
        }
        return mAngularUpperLimits;
    }

    void BulletGeneric6dofConstraint::setBreakingImpulse(float impulse)
    {
        if (mGeneric6DofConstraint)
        {
            mGeneric6DofConstraint->setBreakingImpulseThreshold(impulse);
        }
        else
        {
            mBreakingImpulse = impulse;
        }
    }

    float BulletGeneric6dofConstraint::getBreakingImpulse() const
    {
        if (mGeneric6DofConstraint)
        {
            return mGeneric6DofConstraint->getBreakingImpulseThreshold();
        }
        else
        {
            return mBreakingImpulse;
        }
    }

void BulletGeneric6dofConstraint::updateConstructionInfo()
{
    if (mGeneric6DofConstraint != nullptr)
    {
        return;
    }
    BulletRigidBody* bodyB = ((BulletRigidBody*) owner_object()->getComponent(COMPONENT_TYPE_PHYSICS_RIGID_BODY));

    if (bodyB)
    {
        btRigidBody* rbB = bodyB->getRigidBody();
        btRigidBody* rbA = mRigidBodyA->getRigidBody();
        btVector3    p(mPosition.x, mPosition.y, mPosition.z);
        btMatrix3x3  m(mRotationA[0][0], mRotationA[0][1], mRotationA[0][2],
                       mRotationA[1][0], mRotationA[1][1], mRotationA[1][2],
                       mRotationA[2][0], mRotationA[2][1], mRotationA[2][2]);
        btTransform  fA(m, p);

        p = rbA->getWorldTransform().getOrigin() + p;
        p -= rbB->getWorldTransform().getOrigin();
        m.setValue(mRotationB[0][0], mRotationB[0][1], mRotationB[0][2],
                   mRotationB[1][0], mRotationB[1][1], mRotationB[1][2],
                   mRotationB[2][0], mRotationB[2][1], mRotationB[2][2]);
        btTransform fB(m, p);

        mGeneric6DofConstraint = new btGeneric6DofConstraint(*rbA, *rbB, fA, fB, false);
        mGeneric6DofConstraint->setLinearLowerLimit(Common2Bullet(mLinearLowerLimits));
        mGeneric6DofConstraint->setLinearUpperLimit(Common2Bullet(mLinearUpperLimits));
        mGeneric6DofConstraint->setAngularLowerLimit(Common2Bullet(mAngularLowerLimits));
        mGeneric6DofConstraint->setAngularUpperLimit(Common2Bullet(mAngularUpperLimits));
        mGeneric6DofConstraint->setBreakingImpulseThreshold(mBreakingImpulse);
    }
    else
    {
        BulletJoint* jointB = (BulletJoint*) owner_object()->getComponent(COMPONENT_TYPE_PHYSICS_JOINT);
        if (jointB)
        {
            btMultibodyLink* link = jointB->getLink();
            BulletJoint* jointA = (BulletJoint*) mRigidBodyA;
            btVector3 axis0(mRotationB[0][0], mRotationB[0][1], mRotationB[0][2]);
            btVector3 axis1(mRotationB[1][0], mRotationB[1][1], mRotationB[1][2]);
            btVector3 axis2(mRotationB[2][0], mRotationB[2][1], mRotationB[2][2]);
            btTransform tA;
            btTransform tB;

            jointA->getWorldTransform(tA);
            jointB->getWorldTransform(tB);
            btVector3 posA(tA.getOrigin());
            btVector3 posB(tB.getOrigin());

            link->m_jointType = btMultibodyLink::eSpherical;
            link->m_dVector = posB.normalize();
            link->m_eVector = posA.normalize();
            link->setAxisTop(0, axis0);
            link->setAxisTop(1, axis1);
            link->setAxisTop(2, axis2);
            link->setAxisBottom(0, link->m_dVector.cross(axis0));
            link->setAxisBottom(1, link->m_dVector.cross(axis1));
            link->setAxisBottom(2, link->m_dVector.cross(axis2));
            link->m_dofCount = 3;
        }
    }
}
}