//
// Created by c.bozzetto on 09/06/2017.
//

#ifndef EXTENSIONS_PHYSICS_GENERICCONSTRAINT_H
#define EXTENSIONS_PHYSICS_GENERICCONSTRAINT_H

#include "physics_constraint.h"
#include <glm/glm.hpp>

namespace sxr {

    class PhysicsGenericConstraint : public PhysicsConstraint
    {
    public:
        virtual ~PhysicsGenericConstraint() {}

        virtual const glm::vec3& getLinearUpperLimits() const = 0;
        virtual const glm::vec3& getLinearLowerLimits() const = 0;
        virtual const glm::vec3& getAngularLowerLimits() const = 0;
        virtual const glm::vec3& getAngularUpperLimits() const = 0;
        virtual glm::vec3        getLinearStiffness() const = 0;
        virtual glm::vec3        getAngularStiffness() const = 0;
        virtual glm::vec3        getLinearDamping() const = 0;
        virtual glm::vec3        getAngularDamping() const = 0;
        virtual float            getSpringStiffness(int dof) const = 0;
        virtual float            getSpringDamping(int dof) const = 0;

        virtual void setLinearUpperLimits(float limitX, float limitY, float limitZ) = 0;
        virtual void setLinearLowerLimits(float limitX, float limitY, float limitZ) = 0;
        virtual void setAngularLowerLimits(float limitX, float limitY, float limitZ) = 0;
        virtual void setAngularUpperLimits(float limitX, float limitY, float limitZ) = 0;
        virtual void setSpringStiffness(int dof, float) = 0;
        virtual void setSpringDamping(int dof, float) = 0;
        virtual void setLinearDamping(const glm::vec3& v) = 0;
        virtual void setAngularDamping(const glm::vec3& v) = 0;
        virtual void setLinearStiffness(const glm::vec3& v) = 0;
        virtual void setAngularStiffness(const glm::vec3& v) = 0;

        int getConstraintType() const { return PhysicsConstraint::genericConstraint; }
    };

}

#endif //EXTENSIONS_PHYSICS_GENERICCONSTRAINT_H
