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
    : mConstraint(nullptr),
      mBreakingImpulse(SIMD_INFINITY)
    {
        mBodyA = bodyA;
        mPivotA = pivotA;
        mPivotB = pivotB;
        for (int dof = 0; dof < 6; ++dof)
        {
            mSpringStiffness[dof] = 0;
            mSpringDamping[dof] = 0;
        }
    }

    BulletGeneric6dofConstraint::BulletGeneric6dofConstraint(btGeneric6DofConstraint *constraint)
    : mConstraint(constraint),
      mBreakingImpulse(SIMD_INFINITY)
    {
        mConstraint = constraint;
        mBodyA = static_cast<BulletRigidBody*>(constraint->getRigidBodyA().getUserPointer());
        for (int dof = 0; dof < 6; ++dof)
        {
            mSpringStiffness[dof] = 0;
            mSpringDamping[dof] = 0;
        }
        constraint->setUserConstraintPtr(this);
    }

    BulletGeneric6dofConstraint::BulletGeneric6dofConstraint(btGeneric6DofSpring2Constraint* constraint)
    : mConstraint(constraint),
      mBreakingImpulse(SIMD_INFINITY)
    {
        btTranslationalLimitMotor2* transMotor = constraint->getTranslationalLimitMotor();

        mBodyA = static_cast<BulletRigidBody*>(constraint->getRigidBodyA().getUserPointer());
        mSpringStiffness[0] = transMotor->m_springStiffness.x();
        mSpringStiffness[1] = transMotor->m_springStiffness.y();
        mSpringStiffness[2] = transMotor->m_springStiffness.z();
        mSpringStiffness[3] = constraint->getRotationalLimitMotor(0)->m_springStiffness;
        mSpringStiffness[4] = constraint->getRotationalLimitMotor(1)->m_springStiffness;
        mSpringStiffness[5] = constraint->getRotationalLimitMotor(2)->m_springStiffness;
        mSpringDamping[0] = transMotor->m_springDamping.x();
        mSpringDamping[1] = transMotor->m_springDamping.y();
        mSpringDamping[2] = transMotor->m_springDamping.z();
        mSpringDamping[3] = constraint->getRotationalLimitMotor(0)->m_springDamping;
        mSpringDamping[4] = constraint->getRotationalLimitMotor(1)->m_springDamping;
        mSpringDamping[5] = constraint->getRotationalLimitMotor(2)->m_springDamping;
    }

    BulletGeneric6dofConstraint::BulletGeneric6dofConstraint(btGeneric6DofSpringConstraint* constraint)
    : mConstraint(constraint),
      mBreakingImpulse(SIMD_INFINITY)
    {
        mBodyA = static_cast<BulletRigidBody*>(constraint->getRigidBodyA().getUserPointer());
        for (int dof = 0; dof < 6; ++dof)
        {
            mSpringStiffness[dof] = constraint->getStiffness(dof);
            mSpringDamping[dof] = constraint->getDamping(dof);
        }
    }
    
    BulletGeneric6dofConstraint::~BulletGeneric6dofConstraint()
    {
        if (mConstraint)
        {
            delete mConstraint;
        }
    }

    glm::vec3 BulletGeneric6dofConstraint::getLinearStiffness() const
    {
        glm::vec3 v(mSpringStiffness[0], mSpringStiffness[1], mSpringStiffness[2]);
        return v;
    }

    glm::vec3 BulletGeneric6dofConstraint::getLinearDamping() const
    {
        glm::vec3 v(mSpringDamping[0], mSpringDamping[1], mSpringDamping[2]);
        return v;
    }

    glm::vec3 BulletGeneric6dofConstraint::getAngularStiffness() const
    {
        glm::vec3 v(mSpringStiffness[3], mSpringStiffness[4], mSpringStiffness[5]);
        return v;
    }

    glm::vec3 BulletGeneric6dofConstraint::getAngularDamping() const
    {
        glm::vec3 v(mSpringDamping[3], mSpringDamping[4], mSpringDamping[5]);
        return v;
    }


    float BulletGeneric6dofConstraint::getSpringStiffness(int dof) const
    {
        if ((dof >= 0) && (dof <= 6))
        {
            return mSpringStiffness[dof];
        }
        return 0;
    }

    float BulletGeneric6dofConstraint::getSpringDamping(int dof) const
    {
        if ((dof >= 0) && (dof <= 6))
        {
            return mSpringDamping[dof];
        }
        return 0;
    }

    void BulletGeneric6dofConstraint::setLinearStiffness(const glm::vec3& v)
    {
        btGeneric6DofSpringConstraint* c = dynamic_cast<btGeneric6DofSpringConstraint*>(mConstraint);
        btGeneric6DofSpring2Constraint* sc = dynamic_cast<btGeneric6DofSpring2Constraint*>(mConstraint);

        mSpringStiffness[0] = v.x;
        mSpringStiffness[1] = v.y;
        mSpringStiffness[2] = v.z;
        if (c)
        {
            c->setStiffness(0, v.x);
            c->setStiffness(1, v.y);
            c->setStiffness(2, v.z);
            c->enableSpring(0, (v.x != 0) || (mSpringDamping[0] != 0));
            c->enableSpring(1, (v.y != 0) || (mSpringDamping[1] != 0));
            c->enableSpring(2, (v.z != 0) || (mSpringDamping[2] != 0));
        }
        else if (sc)
        {
            sc->setStiffness(0, v.x);
            sc->setStiffness(1, v.y);
            sc->setStiffness(2, v.z);
            sc->enableSpring(0, (v.x != 0) || (mSpringDamping[0] != 0));
            sc->enableSpring(1, (v.y != 0) || (mSpringDamping[1] != 0));
            sc->enableSpring(2, (v.z != 0) || (mSpringDamping[2] != 0));
        }
    }

    void BulletGeneric6dofConstraint::setAngularStiffness(const glm::vec3& v)
    {
        btGeneric6DofSpringConstraint* c = dynamic_cast<btGeneric6DofSpringConstraint*>(mConstraint);
        btGeneric6DofSpring2Constraint* sc = dynamic_cast<btGeneric6DofSpring2Constraint*>(mConstraint);

        mSpringStiffness[3] = v.x;
        mSpringStiffness[4] = v.y;
        mSpringStiffness[5] = v.z;
        if (c)
        {
            c->setStiffness(3, v.x);
            c->setStiffness(4, v.y);
            c->setStiffness(5, v.z);
            c->enableSpring(3, (v.x != 0) || (mSpringDamping[3] != 0));
            c->enableSpring(4, (v.y != 0) || (mSpringDamping[4] != 0));
            c->enableSpring(5, (v.z != 0) || (mSpringDamping[5] != 0));
        }
        else if (sc)
        {
            sc->setStiffness(3, v.x);
            sc->setStiffness(4, v.y);
            sc->setStiffness(5, v.z);
            sc->enableSpring(3, (v.x != 0) || (mSpringDamping[3] != 0));
            sc->enableSpring(4, (v.y != 0) || (mSpringDamping[4] != 0));
            sc->enableSpring(5, (v.z != 0) || (mSpringDamping[5] != 0));
        }
    }

    void BulletGeneric6dofConstraint::setLinearDamping(const glm::vec3& v)
    {
        btGeneric6DofSpringConstraint* c = dynamic_cast<btGeneric6DofSpringConstraint*>(mConstraint);
        btGeneric6DofSpring2Constraint* sc = dynamic_cast<btGeneric6DofSpring2Constraint*>(mConstraint);

        mSpringDamping[0] = v.x;
        mSpringDamping[1] = v.y;
        mSpringDamping[2] = v.z;
        if (c)
        {
            c->setDamping(0, v.x);
            c->setDamping(1, v.y);
            c->setDamping(2, v.z);
            c->enableSpring(0, (v.x != 0) || (mSpringStiffness[0] == 0));
            c->enableSpring(1, (v.y != 0) || (mSpringStiffness[1] == 0));
            c->enableSpring(2, (v.z != 0) || (mSpringStiffness[2] == 0));
        }
        else if (sc)
        {
            sc->setDamping(0, v.x);
            sc->setDamping(1, v.y);
            sc->setDamping(2, v.z);
            sc->enableSpring(0, (v.x != 0) || (mSpringStiffness[0] == 0));
            sc->enableSpring(1, (v.y != 0) || (mSpringStiffness[1] == 0));
            sc->enableSpring(2, (v.z != 0) || (mSpringStiffness[2] == 0));
        }
    }

    void BulletGeneric6dofConstraint::setAngularDamping(const glm::vec3& v)
    {
        btGeneric6DofSpringConstraint* c = dynamic_cast<btGeneric6DofSpringConstraint*>(mConstraint);
        btGeneric6DofSpring2Constraint* sc = dynamic_cast<btGeneric6DofSpring2Constraint*>(mConstraint);

        mSpringDamping[3] = v.x;
        mSpringDamping[4] = v.y;
        mSpringDamping[5] = v.z;
        if (c)
        {
            c->setDamping(3, v.x);
            c->setDamping(4, v.y);
            c->setDamping(5, v.z);
            c->enableSpring(3, (v.x != 0) || (mSpringStiffness[3] == 0));
            c->enableSpring(4, (v.y != 0) || (mSpringStiffness[4] == 0));
            c->enableSpring(5, (v.z != 0) || (mSpringStiffness[5] == 0));
        }
        else if (sc)
        {
            sc->setDamping(3, v.x);
            sc->setDamping(4, v.y);
            sc->setDamping(5, v.z);
            sc->enableSpring(3, (v.x != 0) || (mSpringStiffness[3] == 0));
            sc->enableSpring(4, (v.y != 0) || (mSpringStiffness[4] == 0));
            sc->enableSpring(5, (v.z != 0) || (mSpringStiffness[5] == 0));
        }
    }
    
    void BulletGeneric6dofConstraint::setSpringStiffness(int dof, float v)
    {
        btGeneric6DofSpringConstraint* c = dynamic_cast<btGeneric6DofSpringConstraint*>(mConstraint);
        btGeneric6DofSpring2Constraint* sc = dynamic_cast<btGeneric6DofSpring2Constraint*>(mConstraint);
        bool enableSpring = (v != 0) && (mSpringDamping[dof] != 0);

        if ((dof > 6) || (dof < 0) ||
            (mSpringStiffness[dof] == v))
        {
            return;
        }
        mSpringStiffness[dof] = v;
        if (c)
        {
            c->setStiffness(dof, v);
            c->enableSpring(dof, enableSpring);
        }
        else if (sc)
        {
            sc->setStiffness(dof, v);
            sc->enableSpring(dof, enableSpring);
        }
    }

    void BulletGeneric6dofConstraint::setSpringDamping(int dof, float v)
    {
        btGeneric6DofSpringConstraint* c = dynamic_cast<btGeneric6DofSpringConstraint*>(mConstraint);
        btGeneric6DofSpring2Constraint* sc = dynamic_cast<btGeneric6DofSpring2Constraint*>(mConstraint);
        bool enableSpring = (v != 0) && (mSpringStiffness[dof] != 0);

        if ((dof > 6) || (dof < 0) ||
            (mSpringDamping[dof] == v))
        {
            return;
        }
        mSpringDamping[dof] = v;
        if (c)
        {
            c->setDamping(dof, v);
            c->enableSpring(dof, enableSpring);
        }
        else if (sc)
        {
            sc->setDamping(dof, v);
            sc->enableSpring(dof, enableSpring);
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

    void BulletGeneric6dofConstraint::setParentBody(PhysicsCollidable* body)
    {
        PhysicsCollidable* bodyA = mBodyA;
        btDynamicsWorld* dw;
        BulletWorld* bw;

        if (body == bodyA)
        {
            return;
        }
        if (bodyA->getType() == COMPONENT_TYPE_PHYSICS_RIGID_BODY)
        {
            BulletRigidBody* rb = static_cast<BulletRigidBody*>(bodyA);
            dw = rb->getPhysicsWorld();

        }
        else if (bodyA->getType() == COMPONENT_TYPE_PHYSICS_JOINT)
        {
            BulletJoint* j = static_cast<BulletJoint*>(bodyA);
            dw = j->getPhysicsWorld();
        }
        mBodyA = body;
        if (mConstraint)
        {
            mBodyA = body;
            if (dw)
            {
                bw = static_cast<BulletWorld*>(dw->getWorldUserInfo());
                dw->removeConstraint(mConstraint);
                delete mConstraint;
                mConstraint = nullptr;
                if (owner_object() != nullptr)
                {
                    sync(bw);
                }
            }
            else
            {
                delete mConstraint;
                mConstraint = nullptr;
            }
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