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

static const char tag[] = "PHYSICS";

namespace sxr {

    BulletGeneric6dofConstraint::BulletGeneric6dofConstraint(
            PhysicsRigidBody *rigidBodyB, float const joint[], float const rotationA[],
            float const rotationB[]) {
        mGeneric6DofConstraint = 0;

        mRigidBodyB = reinterpret_cast<BulletRigidBody*>(rigidBodyB);

        mBreakingImpulse = SIMD_INFINITY;
        mPosition.set(joint);
        mRotationA.set(rotationA);
        mRotationB.set(rotationB);
    }

    BulletGeneric6dofConstraint::BulletGeneric6dofConstraint(btGeneric6DofConstraint *constraint)
    {
        mGeneric6DofConstraint = constraint;
        mRigidBodyB = static_cast<BulletRigidBody*>(constraint->getRigidBodyB().getUserPointer());
        constraint->setUserConstraintPtr(this);
    }

    BulletGeneric6dofConstraint::~BulletGeneric6dofConstraint() {
        if (0 != mGeneric6DofConstraint) {
            delete mGeneric6DofConstraint;
        }
    }

    void BulletGeneric6dofConstraint::setLinearLowerLimits(float limitX, float limitY, float limitZ) {
        if (0 != mGeneric6DofConstraint) {
            mGeneric6DofConstraint->setLinearLowerLimit(btVector3(limitX, limitY, limitZ));
        }
        else {
            mLinearLowerLimits.set(limitX, limitY, limitZ);
        }
    }

    PhysicsVec3 BulletGeneric6dofConstraint::getLinearLowerLimits() const {
        if (0 != mGeneric6DofConstraint) {
            btVector3 t;
            mGeneric6DofConstraint->getLinearLowerLimit(t);
            return PhysicsVec3(t.x(), t.y(), t.z());
        }
        else {
            return mLinearLowerLimits;
        }
    }

    void BulletGeneric6dofConstraint::setLinearUpperLimits(float limitX, float limitY, float limitZ) {
        if (0 != mGeneric6DofConstraint) {
            mGeneric6DofConstraint->setLinearUpperLimit(btVector3(limitX, limitY, limitZ));
        }
        else {
            mLinearUpperLimits.set(limitX, limitY, limitZ);
        }
    }

    PhysicsVec3 BulletGeneric6dofConstraint::getLinearUpperLimits() const {
        if (0 != mGeneric6DofConstraint) {
            btVector3 t;
            mGeneric6DofConstraint->getLinearUpperLimit(t);
            return PhysicsVec3(t.x(), t.y(), t.z());
        }
        else {
            return mLinearUpperLimits;
        }
    }

    void BulletGeneric6dofConstraint::setAngularLowerLimits(float limitX, float limitY, float limitZ) {
        if (0 != mGeneric6DofConstraint) {
            mGeneric6DofConstraint->setAngularLowerLimit(btVector3(limitX, limitY, limitZ));
        }
        else {
            mAngularLowerLimits.set(limitX, limitY, limitZ);
        }
    }

    PhysicsVec3 BulletGeneric6dofConstraint::getAngularLowerLimits() const {
        if (0 != mGeneric6DofConstraint) {
            btVector3 t;
            mGeneric6DofConstraint->getAngularLowerLimit(t);
            return PhysicsVec3(t.x(), t.y(), t.z());
        }
        else {
            return mAngularLowerLimits;
        }
    }

    void BulletGeneric6dofConstraint::setAngularUpperLimits(float limitX, float limitY, float limitZ) {
        if (0 != mGeneric6DofConstraint) {
            mGeneric6DofConstraint->setAngularUpperLimit(btVector3(limitX, limitY, limitZ));
        }
        else {
            mAngularUpperLimits.set(limitX, limitY, limitZ);
        }
    }

    PhysicsVec3 BulletGeneric6dofConstraint::getAngularUpperLimits() const {
        if (0 != mGeneric6DofConstraint) {
            btVector3 t;
            mGeneric6DofConstraint->getAngularUpperLimit(t);
            return PhysicsVec3(t.x(), t.y(), t.z());
        }
        else {
            return mAngularUpperLimits;
        }
    }

    void BulletGeneric6dofConstraint::setBreakingImpulse(float impulse) {
        if (0 != mGeneric6DofConstraint) {
            mGeneric6DofConstraint->setBreakingImpulseThreshold(impulse);
        }
        else {
            mBreakingImpulse = impulse;
        }
    }

    float BulletGeneric6dofConstraint::getBreakingImpulse() const {
        if (0 != mGeneric6DofConstraint) {
            return mGeneric6DofConstraint->getBreakingImpulseThreshold();
        }
        else {
            return mBreakingImpulse;
        }
    }

void BulletGeneric6dofConstraint::updateConstructionInfo()
{
    if (mGeneric6DofConstraint != nullptr)
    {
        return;
    }
    BulletRigidBody* bodyA = ((BulletRigidBody*) owner_object()->getComponent(COMPONENT_TYPE_PHYSICS_RIGID_BODY));

    if (bodyA)
    {
        btRigidBody* rbA = bodyA->getRigidBody();
        btRigidBody* rbB = mRigidBodyB->getRigidBody();
        btVector3    p(mPosition.x, mPosition.y, mPosition.z);
        btMatrix3x3  m(mRotationA.vec[0], mRotationA.vec[1], mRotationA.vec[2],
                       mRotationA.vec[3], mRotationA.vec[4], mRotationA.vec[5],
                       mRotationA.vec[6], mRotationA.vec[7], mRotationA.vec[8]);
        btTransform  fA(m, p);

        p = rbA->getWorldTransform().getOrigin() + p;
        p -= rbB->getWorldTransform().getOrigin();
        m.setValue(mRotationB.vec[0], mRotationB.vec[1], mRotationB.vec[2],
                   mRotationB.vec[3], mRotationB.vec[4], mRotationB.vec[5],
                   mRotationB.vec[6], mRotationB.vec[7], mRotationB.vec[8]);
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
        BulletJoint* jointA = (BulletJoint*) owner_object()->getComponent(COMPONENT_TYPE_PHYSICS_JOINT);
        if (jointA)
        {
            btMultibodyLink* link = jointA->getLink();
            BulletJoint* jointB = (BulletJoint*) mRigidBodyB;
            btVector3 axis0(mRotationB.vec[0], mRotationB.vec[1], mRotationB.vec[2]);
            btVector3 axis1(mRotationB.vec[3], mRotationB.vec[4], mRotationB.vec[5]);
            btVector3 axis2(mRotationB.vec[6], mRotationB.vec[7], mRotationB.vec[8]);
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