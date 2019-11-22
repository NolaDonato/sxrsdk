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


#ifndef PHYSICS_COLLIDABLE_H
#define PHYSICS_COLLIDABLE_H


namespace sxr {
    class PhysicsWorld;

    class PhysicsCollidable : public Component
    {
    public:
        enum SyncOptions
        {
            COLLISION_SHAPE = 1,
            TRANSFORM = 2,
            PROPERTIES = 4,
            IMPORTED = 8,
            ALL = COLLISION_SHAPE | TRANSFORM | PROPERTIES
        };
        PhysicsCollidable(long componentType) : Component(componentType)  { }

        virtual const char* getName() const = 0;

        virtual void setName(const char*) = 0;

        virtual void setMass(float mass) = 0;

        virtual float getMass() const = 0;

        virtual void setFriction(float n)  = 0;

        virtual float getFriction() const = 0;

        virtual const glm::vec3& getScale() const = 0;

        virtual void setScale(const glm::vec3& v) = 0;

        virtual void applyTorque(float x, float y, float z) = 0;

        virtual void sync(int options = 0) = 0;
    };

}
#endif // PHYSICS_COLLIDABLE_H
