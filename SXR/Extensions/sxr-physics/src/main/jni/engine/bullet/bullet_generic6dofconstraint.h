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

class btGeneric6DofConstraint;
namespace sxr {

    class PhysicsRigidBody;
    class BulletRigidBody;

    class BulletGeneric6dofConstraint : public PhysicsGenericConstraint
    {
    public:
        BulletGeneric6dofConstraint(PhysicsCollidable* bodyA, float const joint[],
                                    float const rotationA[], float const rotationB[]);

        BulletGeneric6dofConstraint(btGeneric6DofConstraint *constraint);

        virtual ~BulletGeneric6dofConstraint();

        void setLinearLowerLimits(float limitX, float limitY, float limitZ);

        const glm::vec3& getLinearLowerLimits() const;

        void setLinearUpperLimits(float limitX, float limitY, float limitZ);

        const glm::vec3& getLinearUpperLimits() const;

        void setAngularLowerLimits(float limitX, float limitY, float limitZ);

        const glm::vec3& getAngularLowerLimits() const;

        void setAngularUpperLimits(float limitX, float limitY, float limitZ);

        const glm::vec3& getAngularUpperLimits() const;

        void *getUnderlying() { return mGeneric6DofConstraint;}

        void setBreakingImpulse(float impulse);

        float getBreakingImpulse() const;

        void updateConstructionInfo(PhysicsWorld* world);

    private:

        btGeneric6DofConstraint* mGeneric6DofConstraint;
        PhysicsCollidable*       mRigidBodyA;

        float             mBreakingImpulse;
        mutable glm::vec3 mLinearLowerLimits;
        mutable glm::vec3 mLinearUpperLimits;
        mutable glm::vec3 mAngularLowerLimits;
        mutable glm::vec3 mAngularUpperLimits;
        glm::vec3         mPosition;
        glm::mat3         mRotationA;
        glm::mat3         mRotationB;
    };

}

#endif //EXTENSIONS_BULLET_GENERIC6DOFCONSTRAINT_H
