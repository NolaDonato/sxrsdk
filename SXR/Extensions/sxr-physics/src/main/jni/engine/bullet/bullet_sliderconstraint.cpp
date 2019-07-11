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
// Created by c.bozzetto on 31/05/2017.
//

#include "bullet_rigidbody.h"
#include "bullet_joint.h"
#include "bullet_sliderconstraint.h"
#include "../physics_world.h"
#include <BulletDynamics/ConstraintSolver/btSliderConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodySliderConstraint.h>
#include <LinearMath/btTransform.h>

static const char tag[] = "BulletSliderConstrN";

namespace sxr {

    BulletSliderConstraint::BulletSliderConstraint(PhysicsRigidBody* rigidBodyA)
    {
        mRigidBodyA = reinterpret_cast<BulletRigidBody*>(rigidBodyA);
        mSliderConstraint = 0;

        mBreakingImpulse = SIMD_INFINITY;

        // Default values from btSliderConstraint
        mLowerAngularLimit = 0.0f;
        mUpperAngularLimit = 0.0f;
        mLowerLinearLimit = 1.0f;
        mUpperLinearLimit = -1.0f;
    }

    BulletSliderConstraint::BulletSliderConstraint(btSliderConstraint *constraint)
    {
        mSliderConstraint = constraint;
        mRigidBodyA = static_cast<BulletRigidBody*>(constraint->getRigidBodyA().getUserPointer());
        constraint->setUserConstraintPtr(this);
    }

    BulletSliderConstraint::~BulletSliderConstraint()
    {
        if (mSliderConstraint)
        {
            delete mSliderConstraint;
        }
    }

    void BulletSliderConstraint::setAngularLowerLimit(float limit)
    {
        if (mSliderConstraint)
        {
            mSliderConstraint->setLowerAngLimit(limit);
        }
        else
        {
            mLowerAngularLimit = limit;
        }
    }

    float BulletSliderConstraint::getAngularLowerLimit() const
    {
        if (mSliderConstraint)
        {
            return mSliderConstraint->getLowerAngLimit();
        }
        else
        {
            return mLowerAngularLimit;
        }
    }

    void BulletSliderConstraint::setAngularUpperLimit(float limit)
    {
        if (mSliderConstraint)
        {
            mSliderConstraint->setUpperAngLimit(limit);
        }
        else
        {
            mUpperAngularLimit = limit;
        }
    }

    float BulletSliderConstraint::getAngularUpperLimit() const
    {
        if (mSliderConstraint)
        {
            return mSliderConstraint->getUpperAngLimit();
        }
        else
        {
            return mUpperAngularLimit;
        }
    }

    void BulletSliderConstraint::setLinearLowerLimit(float limit)
    {
        if (mSliderConstraint)
        {
            mSliderConstraint->setLowerLinLimit(limit);
        }
        else
        {
            mLowerLinearLimit = limit;
        }
    }

    float BulletSliderConstraint::getLinearLowerLimit() const
    {
        if (mSliderConstraint)
        {
            return mSliderConstraint->getLowerLinLimit();
        }
        else
        {
            return mLowerLinearLimit;
        }
    }

    void BulletSliderConstraint::setLinearUpperLimit(float limit)
    {
        if (mSliderConstraint)
        {
            mSliderConstraint->setUpperLinLimit(limit);
        }
        else
        {
            mUpperLinearLimit = limit;
        }
    }

    void BulletSliderConstraint::setBreakingImpulse(float impulse)
    {
        if (mSliderConstraint)
        {
            mSliderConstraint->setBreakingImpulseThreshold(impulse);
        }
        else
        {
            mBreakingImpulse = impulse;
        }
    }

    float BulletSliderConstraint::getBreakingImpulse() const
    {
        if (mSliderConstraint)
        {
            return mSliderConstraint->getBreakingImpulseThreshold();
        }
        else
        {
            return mBreakingImpulse;
        }
    }

    float BulletSliderConstraint::getLinearUpperLimit() const
    {
        if (mSliderConstraint)
        {
            return mSliderConstraint->getUpperLinLimit();
        }
        else
        {
            return mUpperLinearLimit;
        }
    }

void BulletSliderConstraint::updateConstructionInfo()
{
    if (mSliderConstraint != nullptr)
    {
        return;
    }
    BulletRigidBody* rigidBodyB = (BulletRigidBody*) this->owner_object()->getComponent(COMPONENT_TYPE_PHYSICS_RIGID_BODY);
    btTransform frameInA = btTransform::getIdentity();
    btTransform frameInB = btTransform::getIdentity();

    if (rigidBodyB)
    {
        btRigidBody* rbB = rigidBodyB->getRigidBody();
        btRigidBody* rbA = mRigidBodyA->getRigidBody();
        mSliderConstraint = new btSliderConstraint(*rbA, *rbB, frameInA, frameInB, true);
        mSliderConstraint->setLowerAngLimit(mLowerAngularLimit);
        mSliderConstraint->setUpperAngLimit(mUpperAngularLimit);
        mSliderConstraint->setLowerLinLimit(mLowerLinearLimit);
        mSliderConstraint->setUpperLinLimit(mUpperLinearLimit);
        mSliderConstraint->setBreakingImpulseThreshold(mBreakingImpulse);
    }
    else
    {
        BulletJoint* jointB = (BulletJoint*) owner_object()->getComponent(COMPONENT_TYPE_PHYSICS_JOINT);
        if (jointB)
        {
            BulletJoint* jointA = (BulletJoint*) mRigidBodyA;
            Transform* transA = owner_object()->transform();
            Transform* transB = jointB->owner_object()->transform();
            btMultibodyLink* link = jointA->getLink();
            glm::vec3 jointAxis = findJointAxis(transA, transB);
            btTransform tA;
            btTransform tB;

            jointA->getWorldTransform(tA);
            jointB->getWorldTransform(tB);
            btVector3 posA(tA.getOrigin());
            btVector3 posB(tB.getOrigin());

            link->m_jointType = btMultibodyLink::ePrismatic;
            link->m_dVector = posB.normalize();
            link->m_eVector = posA.normalize();
            link->setAxisTop(0, 0, 0, 0);
            link->setAxisBottom(0, jointAxis.x, jointAxis.y, jointAxis.z);
            link->m_jointLowerLimit = mLowerLinearLimit;
            link->m_jointUpperLimit = mUpperLinearLimit;
            link->m_dofCount = 1;
        }
    }


}

}