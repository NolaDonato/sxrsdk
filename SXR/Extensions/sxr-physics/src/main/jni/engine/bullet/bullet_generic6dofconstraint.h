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

#ifndef EXTENSIONS_BULLET_GENERIC6DOFCONSTRAINT_H
#define EXTENSIONS_BULLET_GENERIC6DOFCONSTRAINT_H

#include "../physics_genericconstraint.h"
#include "../physics_collidable.h"
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>

class btGeneric6DofSpringConstraint;
class btGeneric6DofSpring2Constraint;
class btGeneric6DofConstraint;
class btTypedConstraint;

namespace sxr {

    class PhysicsRigidBody;
    class BulletRigidBody;

    class BulletGeneric6dofConstraint : public PhysicsGenericConstraint
    {
    public:
        BulletGeneric6dofConstraint(PhysicsCollidable* bodyA, const glm::vec3& pivotA, const glm::vec3& pivotB);
        BulletGeneric6dofConstraint(btGeneric6DofConstraint *constraint);
        BulletGeneric6dofConstraint(btGeneric6DofSpring2Constraint *constraint);
        BulletGeneric6dofConstraint(btGeneric6DofSpringConstraint *constraint);
        virtual ~BulletGeneric6dofConstraint();

        virtual const glm::vec3& getLinearLowerLimits() const;
        virtual const glm::vec3& getLinearUpperLimits() const;
        virtual const glm::vec3& getAngularUpperLimits() const;
        virtual const glm::vec3& getAngularLowerLimits() const;
        virtual glm::vec3        getLinearStiffness() const;
        virtual glm::vec3        getAngularStiffness() const;
        virtual glm::vec3        getLinearDamping() const;
        virtual glm::vec3        getAngularDamping() const;
        virtual float getSpringStiffness(int dof) const;
        virtual float getSpringDamping(int dof) const;
        virtual void* getUnderlying() { return mConstraint; }
        virtual float getBreakingImpulse() const;

        virtual void  setLinearLowerLimits(float limitX, float limitY, float limitZ);
        virtual void  setLinearUpperLimits(float limitX, float limitY, float limitZ);
        virtual void  setAngularLowerLimits(float limitX, float limitY, float limitZ);
        virtual void  setAngularUpperLimits(float limitX, float limitY, float limitZ);
        virtual void  setBreakingImpulse(float impulse);
        virtual void  setSpringStiffness(int dof, float);
        virtual void  setSpringDamping(int dof, float);
        virtual void  setLinearDamping(const glm::vec3& v);
        virtual void  setAngularDamping(const glm::vec3& v);
        virtual void  setLinearStiffness(const glm::vec3& v);
        virtual void  setAngularStiffness(const glm::vec3& v);
        virtual void  setParentBody(PhysicsCollidable* body);

        virtual void  sync(PhysicsWorld *world);
        virtual void  addToWorld(PhysicsWorld*);
        virtual void  removeFromWorld(PhysicsWorld*);

    private:

        btTypedConstraint* mConstraint;
        float              mBreakingImpulse;
        mutable glm::vec3  mLinearLowerLimits;
        mutable glm::vec3  mLinearUpperLimits;
        mutable glm::vec3  mAngularLowerLimits;
        mutable glm::vec3  mAngularUpperLimits;
        mutable float      mSpringStiffness[6];
        mutable float      mSpringDamping[6];
    };

}

#endif //EXTENSIONS_BULLET_GENERIC6DOFCONSTRAINT_H
