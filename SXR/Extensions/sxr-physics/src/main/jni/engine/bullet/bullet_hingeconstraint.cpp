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
// Created by c.bozzetto on 30/05/2017.
//

#include "bullet_hingeconstraint.h"
#include "bullet_rigidbody.h"
#include "bullet_joint.h"
#include "bullet_world.h"
#include "bullet_sxr_utils.h"

#include <BulletDynamics/ConstraintSolver/btHingeConstraint.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>

const char tag[] = "BulletHingeConstrN";

namespace sxr {

    BulletHingeConstraint::BulletHingeConstraint(PhysicsCollidable* bodyA,
                                                 const glm::vec3& pivotA, const glm::vec3& pivotB,
                                                 const glm::vec3 axis)
    {
        mConstraint = 0;
        mBodyA = bodyA;
        mBreakingImpulse = SIMD_INFINITY;
        mPivotA = pivotA;
        mPivotB = pivotB;
        mAxis = axis;

        // By default angular limit is inactive
        mTempLower = 2.0f;
        mTempUpper = 0.0f;
    }

    BulletHingeConstraint::BulletHingeConstraint(btHingeConstraint *constraint)
    {
        mConstraint = constraint;
        btTransform transA = constraint->getAFrame();
        btVector3 axis = transA.getBasis().getColumn(0);
        mBodyA = static_cast<PhysicsCollidable*>(constraint->getRigidBodyA().getUserPointer());
        mAxis.x = axis.x();
        mAxis.y = axis.y();
        mAxis.z = axis.z();
        constraint->setUserConstraintPtr(this);
    }

    BulletHingeConstraint::~BulletHingeConstraint()
    {
        if (mConstraint)
        {
            delete mConstraint;
        }
    }

    void BulletHingeConstraint::setLimits(float lower, float upper)
    {
        if (mConstraint)
        {
            mConstraint->setLimit(lower, upper);
        }
        mTempLower = lower;
        mTempUpper = upper;
    }

    float BulletHingeConstraint::getLowerLimit() const
    {
        if (mConstraint)
        {
            return mConstraint->getLowerLimit();
        }
        else
        {
            return mTempLower;
        }
    }

    float BulletHingeConstraint::getUpperLimit() const
    {
        if (mConstraint)
        {
            return mConstraint->getUpperLimit();
        }
        else
        {
            return mTempUpper;
        }
    }

    void BulletHingeConstraint::setBreakingImpulse(float impulse)
    {
        if (mConstraint)
        {
            mConstraint->setBreakingImpulseThreshold(impulse);
        }
        else
        {
            mBreakingImpulse = impulse;
        }
    }

    float BulletHingeConstraint::getBreakingImpulse() const
    {
        if (mConstraint)
        {
            return mConstraint->getBreakingImpulseThreshold();
        }
        else
        {
            return mBreakingImpulse;
        }
    }

    void BulletHingeConstraint::setParentBody(PhysicsCollidable* body)
    {
        PhysicsCollidable* bodyA = mBodyA;
        BulletRigidBody* rb = static_cast<BulletRigidBody*>(bodyA);
        BulletWorld* bw;
        btDynamicsWorld* dw = rb->getPhysicsWorld();

        if (body == bodyA)
        {
            return;
        }
        if (mConstraint)
        {
            if (dw)
            {
                bw = static_cast<BulletWorld*>(dw->getWorldUserInfo());
                dw->removeConstraint(mConstraint);
            }
            delete mConstraint;
            mConstraint = nullptr;
            mBodyA = body;
            sync(bw);
        }
        else
        {
            mBodyA = body;
        }
    }

    void BulletHingeConstraint::sync(PhysicsWorld *world)
    {
        if (mConstraint != nullptr)
        {
            return;
        }
        BulletRigidBody* bodyB = (BulletRigidBody*) owner_object()->getComponent(COMPONENT_TYPE_PHYSICS_RIGID_BODY);
        if (bodyB)
        {
            btRigidBody* rbB = bodyB->getRigidBody();
            btRigidBody* rbA = static_cast<BulletRigidBody*>(mBodyA)->getRigidBody();
            btVector3    pA(mPivotA.x, mPivotA.y, mPivotA.z);
            btVector3    pB(mPivotB.x, mPivotB.y, mPivotB.z);
            btVector3    axisA(mAxis.x, mAxis.y, mAxis.z);
            btTransform  worldFrameA = convertTransform2btTransform(mBodyA->owner_object()->transform());
            btTransform  worldFrameB = convertTransform2btTransform(owner_object()->transform());
            btTransform  localFrameA = worldFrameB.inverse() * worldFrameA;
            btTransform  localFrameB = worldFrameA.inverse() * worldFrameB;

            axisA.normalize();
            localFrameA.setOrigin(pA);
            localFrameB.setOrigin(pB);
            btVector3 axisB = localFrameB.getBasis() * axisA;

            axisB.normalize();
            mConstraint = new btHingeConstraint(*rbA, *rbB, pA, pB, axisA, axisB, true);
            mConstraint->setLimit(mTempLower, mTempUpper);
            mConstraint->setBreakingImpulseThreshold(mBreakingImpulse);
        }
    }

    void BulletHingeConstraint::addToWorld(PhysicsWorld* w)
    {
        BulletWorld* bw = static_cast<BulletWorld*>(w);

        if (mConstraint)
        {
            bw->getPhysicsWorld()->addConstraint(mConstraint, true);
        }
    }

    void BulletHingeConstraint::removeFromWorld(PhysicsWorld* w)
    {
        BulletWorld* bw = static_cast<BulletWorld*>(w);

        if (mConstraint)
        {
            bw->getPhysicsWorld()->removeConstraint(mConstraint);
        }
    }
}