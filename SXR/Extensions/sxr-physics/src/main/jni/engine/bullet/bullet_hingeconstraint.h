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

#ifndef EXTENSIONS_BULLET_HINGECONSTRAINT_H
#define EXTENSIONS_BULLET_HINGECONSTRAINT_H

#include "../physics_hingeconstraint.h"
#include "../physics_collidable.h"
#include <glm/vec3.hpp>

class btHingeConstraint;

namespace sxr {

    class PhysicsRigidBody;
    class BulletRigidBody;
    class BulletHingeConstraint : public PhysicsHingeConstraint
    {

    public:
        BulletHingeConstraint(PhysicsCollidable* bodyA,
                const glm::vec3& pivotA, const glm::vec3& pivotB,
                const glm::vec3 axis);

        BulletHingeConstraint(btHingeConstraint *constraint);
        virtual ~BulletHingeConstraint();

        virtual const glm::vec3& getAxis() { return mAxis; }
        virtual float getLowerLimit() const;
        virtual float getUpperLimit() const;
        virtual float getBreakingImpulse() const;
        virtual void* getUnderlying() { return mConstraint; }
        virtual void  setLimits(float lower, float upper);
        virtual void  setBreakingImpulse(float impulse);
        virtual void  sync(PhysicsWorld *world);
        virtual void  addToWorld(PhysicsWorld*);
        virtual void  removeFromWorld(PhysicsWorld*);
        virtual void  setParentBody(PhysicsCollidable* body);

    private:
        btHingeConstraint* mConstraint;
        float     mBreakingImpulse;
        float     mTempLower;
        float     mTempUpper;
        glm::vec3 mAxis;
    };
}
#endif //EXTENSIONS_BULLET_HINGECONSTRAINT_H
