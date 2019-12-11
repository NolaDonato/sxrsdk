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

package com.samsungxr.physics;

import com.samsungxr.SXRContext;
import com.samsungxr.SXRTransform;

import org.joml.Vector3f;

/**
 * Created by c.bozzetto on 09/06/2017.
 */

/**
 * Represents a generic constraint for two {@linkplain SXRRigidBody rigid bodies} or
 * {@linkplain SXRPhysicsJoint joints} that are linked by a point related to the first one.
 * The second one is the owner of the constraint). Though it can be moved the
 * first body can be referred as "fixed" since it will keep same distance and rotation from this
 * joint point while the second is the "moving" because one can explicitly set restriction for each
 * translation and rotation axis.
 */
public class SXRGenericConstraint extends SXRConstraint
{
    /**
     * Construct a new instance of a generic constraint.
     *
     * @param ctx        the context of the app
     * @param bodyA      the "fixed" body (not the owner) in this constraint
     * @param pivotA     the pivot point (x, y and z coordinates) in this constraint
     *                   relative to "fixed" body
     * @param pivotB     the pivot point (x, y and z coordinates) in this constraint
     *                   relative to owner
     */
    public SXRGenericConstraint(SXRContext ctx, SXRPhysicsCollidable bodyA, final float pivotA[], final float pivotB[])
    {
        this(ctx, Native3DGenericConstraint.ctor(bodyA.getNative(),
                                                 pivotA[0], pivotA[1], pivotA[2],
                                                 pivotB[0], pivotB[1], pivotB[2]));
        mBodyA = bodyA;
    }

    /**
     * Construct a new instance of a generic constraint.
     *
     * @param ctx        the context of the app
     * @param bodyA      the "fixed" body (not the owner) in this constraint
     * @param pivotA     the pivot point (x, y and z coordinates) in this constraint
     *                   relative to "fixed" body
     * @param pivotB     the pivot point (x, y and z coordinates) in this constraint
     *                   relative to owner
     */
    public SXRGenericConstraint(SXRContext ctx, SXRPhysicsCollidable bodyA, final Vector3f pivotA, final Vector3f pivotB)
    {
        this(ctx, Native3DGenericConstraint.ctor(bodyA.getNative(),
                                                 pivotA.x, pivotA.y, pivotA.z,
                                                 pivotB.x, pivotB.y, pivotB.z));
        mBodyA = bodyA;
    }

    /**
     * Construct a new instance of a generic constraint.
     *
     * @param ctx        the context of the app
     * @param bodyA      the "fixed" body (not the owner) in this constraint
     * @param pivotB     the pivot point (x, y and z coordinates) in this constraint
     *                   relative to owner
     */
    public SXRGenericConstraint(SXRContext ctx, SXRPhysicsCollidable bodyA, final Vector3f pivotB)
    {
        this(ctx, Native3DGenericConstraint.ctor(bodyA.getNative(),
                                                 0, 0, 0,
                                                 pivotB.x, pivotB.y, pivotB.z));
        mBodyA = bodyA;
    }

    /**
     * Construct a new instance of a generic constraint.
     *
     * @param ctx        the context of the app
     * @param bodyA      the "fixed" body (not the owner) in this constraint
     */
    public SXRGenericConstraint(SXRContext ctx, SXRPhysicsCollidable bodyA)
    {
        this(ctx, Native3DGenericConstraint.ctor(bodyA.getNative(),
                                                 0, 0, 0,
                                                 0, 0, 0));
        mBodyA = bodyA;
    }

    /** Used only by {@link SXRPhysicsLoader} */
    SXRGenericConstraint(SXRContext gvrContext, long nativeConstraint)
    {
        super(gvrContext, nativeConstraint);
    }

    /**
     * Sets the lower limits for the "moving" body translation relative to joint point.
     *
     * @param limitX the X axis lower translation limit
     * @param limitY the Y axis lower translation limit
     * @param limitZ the Z axis lower translation limit
     */
    public void setLinearLowerLimits(float limitX, float limitY, float limitZ)
    {
        Native3DGenericConstraint.setLinearLowerLimits(getNative(), limitX, limitY, limitZ);
    }

    /**
     * Gets the lower limits for the "moving" body translation relative to joint point.
     *
     * @return an array containing the lower translation limits for each (X, Y and Z) axis.
     */
    public float[] getLinearLowerLimits()
    {
        return Native3DGenericConstraint.getLinearLowerLimits(getNative());
    }

    /**
     * Sets the upper limits for the "moving" body translation relative to joint point.
     *
     * @param limitX the X upper lower translation limit
     * @param limitY the Y upper lower translation limit
     * @param limitZ the Z upper lower translation limit
     */
    public void setLinearUpperLimits(float limitX, float limitY, float limitZ)
    {
        Native3DGenericConstraint.setLinearUpperLimits(getNative(), limitX, limitY, limitZ);
    }

    /**
     * Gets the upper limits for the "moving" body translation relative to joint point.
     *
     * @return an array containing the upper translation limits for each (X, Y and Z) axis.
     */
    public float[] getLinearUpperLimits()
    {
        return Native3DGenericConstraint.getLinearUpperLimits(getNative());
    }

    /**
     * Sets the lower limits for the "moving" body rotation relative to joint point.
     *
     * @param limitX the X axis lower rotation limit (in radians)
     * @param limitY the Y axis lower rotation limit (in radians)
     * @param limitZ the Z axis lower rotation limit (in radians)
     */
    public void setAngularLowerLimits(float limitX, float limitY, float limitZ)
    {
        if ((limitX < -(0.00001f + Math.PI)) ||
            (limitY < -(0.00001f + (Math.PI / 2.0f))) ||
            (limitZ < -(0.00001f + Math.PI)))
        {
            throw new IllegalArgumentException("Angular limits out of range");
        }
        Native3DGenericConstraint.setAngularLowerLimits(getNative(), limitX, limitY, limitZ);
    }

    /**
     * Gets the lower limits for the "moving" body rotation relative to joint point.
     *
     * @return an array containing the lower rotation limits for each (X, Y and Z) axis.
     */
    public float[] getAngularLowerLimits()
    {
        return Native3DGenericConstraint.getAngularLowerLimits(getNative());
    }

    /**
     * Sets the upper limits for the "moving" body rotation relative to joint point.
     *
     * @param limitX the X axis upper rotation limit (in radians)
     * @param limitY the Y axis upper rotation limit (in radians)
     * @param limitZ the Z axis upper rotation limit (in radians)
     */
    public void setAngularUpperLimits(float limitX, float limitY, float limitZ)
    {
        if ((limitX > (Math.PI + 0.00001f)) ||
            (limitY > (Math.PI / 2.0f + 0.00001f)) ||
            (limitZ > (Math.PI + 0.00001f)))
        {
            throw new IllegalArgumentException("Angular limits out of range");
        }
        Native3DGenericConstraint.setAngularUpperLimits(getNative(), limitX, limitY, limitZ);
    }

    /**
     * Gets the upper limits for the "moving" body rotation relative to joint point.
     *
     * @return an array containing the upper rotation limits for each (X, Y and Z) axis.
     */
    public float[] getAngularUpperLimits()
    {
        return Native3DGenericConstraint.getAngularUpperLimits(getNative());
    }

    /**
     * Set the spring stiffness for one degree of freedom of the spring constraint.
     * @param dof       degree of freedom (0 - 5)
     *                  0 - 2 is linear stiffness
     *                  3 - 5 is angular stiffness
     * @param stiffness stiffness value (0 to 1)
     */
    public void setSpringStiffness(int dof, float stiffness)
    {
        Native3DGenericConstraint.setSpringStiffness(getNative(), dof, stiffness);
    }

    /**
     * Set the spring damping for one degree of freedom of the spring constraint.
     * @param dof     degree of freedom (0 - 5)
     *                  0 - 2 is linear damping
     *                  3 - 5 is angular damping
     * @param damping damping value
     */
    public void setSpringDamping(int dof, float damping)
    {
        Native3DGenericConstraint.setSpringStiffness(getNative(), dof, damping);
    }

    /**
     * Sets the linear spring stiffness of the spring constraint.
     * The stiffness for degrees of freedom 0 thru 2 are affected.
     * @param s0 spring stiffness for linear degree of freedom 0
     * @param s1 spring stiffness for linear degree of freedom 1
     * @param s2 spring stiffness for linear degree of freedom 2
     */
    public void setLinearStiffness(float s0, float s1, float s2)
    {
        Native3DGenericConstraint.setLinearStiffness(getNative(), s0, s1, s2);
    }

    /**
     * Sets the linear spring damping of the spring constraint.
     * The damping for degrees of freedom 0 thru 2 are affected.
     * @param d0 spring damping for angular degree of freedom 0
     * @param d1 spring damping for angular degree of freedom 1
     * @param d2 spring damping for angular degree of freedom 2
     */
    public void setLinearDamping(float d0, float d1, float d2)
    {
        Native3DGenericConstraint.setLinearDamping(getNative(), d0, d1, d2);
    }

    /**
     * Sets the angular spring stiffness of the spring constraint.
     * The stiffness for degrees of freedom 3 thru 5 are affected.
     * @param s0 spring stiffness for angular degree of freedom 3
     * @param s1 spring stiffness for angular degree of freedom 4
     * @param s2 spring stiffness for angular degree of freedom 5
     */
    public void setAngularStiffness(float s0, float s1, float s2)
    {
        Native3DGenericConstraint.setLinearStiffness(getNative(), s0, s1, s2);
    }

    /**
     * Sets the angular spring damping of the spring constraint.
     * The damping for degrees of freedom 3 thru 5 are affected.
     * @param d0 spring damping for angular degree of freedom 3
     * @param d1 spring damping for angular degree of freedom 4
     * @param d2 spring damping for angular degree of freedom 5
     */
    public void setAngularDamping(float d0, float d1, float d2)
    {
        Native3DGenericConstraint.setLinearDamping(getNative(), d0, d1, d2);
    }

    /**
     * Return the spring stiffness for the 3 linear degrees of freedom
     * (dof 0 thru 2).
     * @return array with linear spring stiffness for dof 0 - 2
     */
    public float[] getLinearStiffness()
    {
        return Native3DGenericConstraint.getLinearStiffness(getNative());
    }

    /**
     * Return the spring stiffness for the 3 angular degrees of freedom
     * (dof 3 thru 5).
     * @return array with angular spring stiffness for dof 3 - 5
     */
    public float[] getAngularStiffness()
    {
        return Native3DGenericConstraint.getAngularStiffness(getNative());
    }

    /**
     * Return the spring damping for the 3 linear degrees of freedom
     * (dof 0 thru 2).
     * @return array with linear spring damping for dof 0 - 2
     */
    public float[] getLinearDamping()
    {
        return Native3DGenericConstraint.getLinearDamping(getNative());
    }

    /**
     * Return the spring damping for the 3 angular degrees of freedom
     * (dof 3 thru 5).
     * @return array with angular spring damping for dof 3 - 5
     */
    public float[] getAngularDamping()
    {
        return Native3DGenericConstraint.getAngularDamping(getNative());
    }
}

class Native3DGenericConstraint
{
    static native long ctor(long rigidBodyA,
                            float pivotAx, float pivotAy, float pivotAz,
                            float pivotBx, float pivotBy, float pivotBz);

    static native float[] getLinearLowerLimits(long jconstr);
    static native float[] getLinearUpperLimits(long jconstr);
    static native float[] getAngularLowerLimits(long jconstr);
    static native float[] getAngularUpperLimits(long jconstr);
    static native float[] getLinearStiffness(long jconstr);
    static native float[] getAngularStiffness(long jconstr);
    static native float[] getLinearDamping(long jconstr);
    static native float[] getAngularDamping(long jconstr);

    static native void setLinearLowerLimits(long jconstr, float limX, float limY, float limZ);
    static native void setLinearUpperLimits(long jconstr, float limX, float limY, float limZ);
    static native void setAngularLowerLimits(long jconstr, float limX, float limY, float limZ);
    static native void setAngularUpperLimits(long jconstr, float limX, float limY, float limZ);
    static native void setSpringStiffness(long jconstr, int dof, float stiffness);
    static native void setSpringDamping(long jconstr, int dof, float damping);
    static native void setLinearStiffness(long jconstr, float s0, float s1, float s2);
    static native void setAngularStiffness(long jconstr, float s0, float s1, float s2);
    static native void setAngularDamping(long jconstr, float d0, float d1, float d2);
    static native void setLinearDamping(long jconstr, float d0, float d1, float d2);
}