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
#include "bullet_world.h"
#include "bullet_rigidbody.h"
#include "bullet_sxr_utils.h"

#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h>
#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#include "glm/gtc/type_ptr.hpp"

static const char tag[] = "PHYSICS";

namespace sxr {

    BulletGeneric6dofConstraint::BulletGeneric6dofConstraint(PhysicsCollidable* bodyA, const glm::vec3& pivotA, const glm::vec3& pivotB)
    {
        mConstraint = 0;

        mBodyA = bodyA;
        mBreakingImpulse = SIMD_INFINITY;
        mPivotA = pivotA;
        mPivotB = pivotB;
    }

    BulletGeneric6dofConstraint::BulletGeneric6dofConstraint(btGeneric6DofConstraint *constraint)
    {
        mConstraint = constraint;
        mBodyA = static_cast<BulletRigidBody*>(constraint->getRigidBodyA().getUserPointer());
        constraint->setUserConstraintPtr(this);
    }

    BulletGeneric6dofConstraint::BulletGeneric6dofConstraint(btGeneric6DofSpring2Constraint* constraint)
    {
        mConstraint = constraint;
        mBodyA = static_cast<BulletRigidBody*>(constraint->getRigidBodyA().getUserPointer());
        constraint->setUserConstraintPtr(this);
    }

    BulletGeneric6dofConstraint::~BulletGeneric6dofConstraint()
    {
        if (mConstraint)
        {
            delete mConstraint;
        }
    }

    void BulletGeneric6dofConstraint::setLinearLowerLimits(float limitX, float limitY, float limitZ)
    {
        btGeneric6DofConstraint* c = dynamic_cast<btGeneric6DofConstraint*>(mConstraint);
        btGeneric6DofSpring2Constraint* sc = dynamic_cast<btGeneric6DofSpring2Constraint*>(mConstraint);

        mLinearLowerLimits = glm::vec3(limitX, limitY, limitZ);
        if (c)
        {
            c->setLinearLowerLimit(btVector3(limitX, limitY, limitZ));
        }
        else if (sc)
        {
            sc->setLinearLowerLimit(btVector3(limitX, limitY, limitZ));
        }
    }

    const glm::vec3& BulletGeneric6dofConstraint::getLinearLowerLimits() const
    {
        btVector3 t;
        btGeneric6DofConstraint* c = dynamic_cast<btGeneric6DofConstraint*>(mConstraint);
        btGeneric6DofSpring2Constraint* sc = dynamic_cast<btGeneric6DofSpring2Constraint*>(mConstraint);

        if (c)
        {
            c->getLinearLowerLimit(t);
        }
        else if (sc)
        {
            sc->getLinearLowerLimit(t);
        }
        else
        {
           return mLinearLowerLimits;
        }
        mLinearLowerLimits.x = t.x();
        mLinearLowerLimits.y = t.y();
        mLinearLowerLimits.z = t.z();
        return mLinearLowerLimits;
    }

    void BulletGeneric6dofConstraint::setLinearUpperLimits(float limitX, float limitY, float limitZ)
    {
        btGeneric6DofConstraint* c = dynamic_cast<btGeneric6DofConstraint*>(mConstraint);
        btGeneric6DofSpring2Constraint* sc = dynamic_cast<btGeneric6DofSpring2Constraint*>(mConstraint);

        mLinearUpperLimits = glm::vec3(limitX, limitY, limitZ);
        if (c)
        {
            c->setLinearUpperLimit(btVector3(limitX, limitY, limitZ));
        }
        else if (sc)
        {
            sc->setLinearUpperLimit(btVector3(limitX, limitY, limitZ));
        }
    }

    const glm::vec3& BulletGeneric6dofConstraint::getLinearUpperLimits() const
    {
        btVector3 t;
        btGeneric6DofConstraint* c = dynamic_cast<btGeneric6DofConstraint*>(mConstraint);
        btGeneric6DofSpring2Constraint* sc = dynamic_cast<btGeneric6DofSpring2Constraint*>(mConstraint);

        if (c)
        {
            c->getLinearUpperLimit(t);
        }
        else if (sc)
        {
            sc->getLinearUpperLimit(t);
        }
        else
        {
            return mLinearUpperLimits;
        }
        mLinearUpperLimits.x = t.x();
        mLinearUpperLimits.y = t.y();
        mLinearUpperLimits.z = t.z();
        return mLinearUpperLimits;
    }

    void BulletGeneric6dofConstraint::setAngularLowerLimits(float limitX, float limitY, float limitZ)
    {
        btGeneric6DofConstraint* c = dynamic_cast<btGeneric6DofConstraint*>(mConstraint);
        btGeneric6DofSpring2Constraint* sc = dynamic_cast<btGeneric6DofSpring2Constraint*>(mConstraint);

        mAngularLowerLimits = glm::vec3(limitX, limitY, limitZ);
        if (c)
        {
            c->setAngularUpperLimit(btVector3(limitX, limitY, limitZ));
        }
        else if (sc)
        {
            sc->setAngularUpperLimit(btVector3(limitX, limitY, limitZ));
        }
    }

    const glm::vec3&  BulletGeneric6dofConstraint::getAngularLowerLimits() const
    {
        btVector3 t;
        btGeneric6DofConstraint* c = dynamic_cast<btGeneric6DofConstraint*>(mConstraint);
        btGeneric6DofSpring2Constraint* sc = dynamic_cast<btGeneric6DofSpring2Constraint*>(mConstraint);

        if (c)
        {
            c->getAngularLowerLimit(t);
        }
        else if (sc)
        {
            sc->getAngularLowerLimit(t);
        }
        else
        {
            return mAngularLowerLimits;
        }
        mAngularLowerLimits.x = t.x();
        mAngularLowerLimits.y = t.y();
        mAngularLowerLimits.z = t.z();
        return mAngularLowerLimits;
    }

    void BulletGeneric6dofConstraint::setAngularUpperLimits(float limitX, float limitY, float limitZ)
    {
        btGeneric6DofConstraint* c = dynamic_cast<btGeneric6DofConstraint*>(mConstraint);
        btGeneric6DofSpring2Constraint* sc = dynamic_cast<btGeneric6DofSpring2Constraint*>(mConstraint);

        mAngularUpperLimits = glm::vec3(limitX, limitY, limitZ);
        if (c)
        {
            c->setAngularUpperLimit(btVector3(limitX, limitY, limitZ));
        }
        else if (sc)
        {
            sc->setAngularUpperLimit(btVector3(limitX, limitY, limitZ));
        }
    }

    const glm::vec3& BulletGeneric6dofConstraint::getAngularUpperLimits() const
    {
        btVector3 t;
        btGeneric6DofConstraint* c = dynamic_cast<btGeneric6DofConstraint*>(mConstraint);
        btGeneric6DofSpring2Constraint* sc = dynamic_cast<btGeneric6DofSpring2Constraint*>(mConstraint);

        if (c)
        {
            c->getAngularUpperLimit(t);
        }
        else if (sc)
        {
            sc->getAngularUpperLimit(t);
        }
        else
        {
            return mAngularUpperLimits;
        }
        mAngularUpperLimits.x = t.x();
        mAngularUpperLimits.y = t.y();
        mAngularUpperLimits.z = t.z();
        return mAngularUpperLimits;
    }

    void BulletGeneric6dofConstraint::setBreakingImpulse(float impulse)
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

    float BulletGeneric6dofConstraint::getBreakingImpulse() const
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

    void BulletGeneric6dofConstraint::sync(PhysicsWorld *world)
    {
        if (mConstraint != nullptr)
        {
            return;
        }
        BulletRigidBody* bodyB = ((BulletRigidBody*) owner_object()->getComponent(COMPONENT_TYPE_PHYSICS_RIGID_BODY));

        if (bodyB)
        {
            btRigidBody* rbB = bodyB->getRigidBody();
            btRigidBody* rbA = static_cast<BulletRigidBody*>(mBodyA)->getRigidBody();
            btVector3    pA(mPivotA.x, mPivotA.y, mPivotA.z);
            btVector3    pB(mPivotB.x, mPivotB.y, mPivotB.z);
            btTransform  worldFrameA = convertTransform2btTransform(mBodyA->owner_object()->transform());
            btTransform  worldFrameB = convertTransform2btTransform(owner_object()->transform());
            btTransform  localFrameA = worldFrameB.inverse() * worldFrameA;
            btTransform  localFrameB = worldFrameA.inverse() * worldFrameB;

            localFrameA.setOrigin(pA);
            localFrameB.setOrigin(pB);
            btGeneric6DofSpringConstraint* constraint = new btGeneric6DofSpringConstraint(*rbA, *rbB, localFrameA, localFrameB, false);
            constraint->setLinearLowerLimit(Common2Bullet(mLinearLowerLimits));
            constraint->setLinearUpperLimit(Common2Bullet(mLinearUpperLimits));
            constraint->setAngularLowerLimit(Common2Bullet(mAngularLowerLimits));
            constraint->setAngularUpperLimit(Common2Bullet(mAngularUpperLimits));
            constraint->setBreakingImpulseThreshold(mBreakingImpulse);
            mConstraint = constraint;
        }
    }

    void BulletGeneric6dofConstraint::addToWorld(PhysicsWorld* w)
    {
        BulletWorld* bw = static_cast<BulletWorld*>(w);

        if (mConstraint)
        {
            bw->getPhysicsWorld()->addConstraint(mConstraint, true);
        }
    }

    void BulletGeneric6dofConstraint::removeFromWorld(PhysicsWorld* w)
    {
        BulletWorld* bw = static_cast<BulletWorld*>(w);

        if (mConstraint)
        {
            bw->getPhysicsWorld()->removeConstraint(mConstraint);
        }
    }
}