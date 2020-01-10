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

import com.samsungxr.SXRCollider;
import com.samsungxr.SXRComponent;
import com.samsungxr.SXRContext;
import com.samsungxr.SXRNode;
import com.samsungxr.SXRTransform;
import com.samsungxr.animation.SXRPoseMapper;
import com.samsungxr.animation.SXRSkeleton;

import org.joml.Vector3f;

import java.lang.annotation.Native;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * Represents a joint in an articulated body.
 * <p>
 * Articulated bodies are implemented using the Bullet
 * Featherstone multibody simulator. A joint can only
 * be added to a simulation world in which multibody
 * support has been enabled.
   A joint can be static or dynamic.
 * Static joints don't move at all, dynamic joints are moved by
 * the physics engine.
 * <p>
 * Joints also have mass and respond to physical forces.
 * To participate in collisions, a joint must have a
 * {@link SXRCollider} component attached to its owner
 * which describes the shape of the rigid body.
 * <p>
 * The joint component is attached to a {@linkplain com.samsungxr.SXRNode node}
 * and uses the transform of its owner object, updating it if the joint is dynamic.
 * Before attaching a joint to a node, make sure the node has the proper
 * position and orientation. You cannot attach a rigid body to a node
 * unless it is in the scene and has a collider component.
 * <p>
 * Unlike rigid bodies, joints are connected in a hierarchy which is
 * not necessarily the same as the hierarchy of the corresponding owner nodes.
 * Each joint has a type which constrains it with respect to its parent.
 * Currently fixed, ball, hinge and slider joints are supported.
 * Constraints between rigid bodies and joints are not currently
 * supported (although the Bullet multibody simulator can do it).
 * </p>
 * @see SXRNode
 * @see SXRCollider
 * @see SXRWorld
 */
public class SXRPhysicsJoint extends SXRPhysicsCollidable
{
    protected SXRSkeleton             mSkeleton = null;
    private final SXRPhysicsContext   mPhysicsContext;

    /**
     * Joint is fixed and does not move.
     */
    static final int FIXED = 1;

    /**
     * Joint rotates spherically around its parent (ball joint).
     */
    static final int SPHERICAL = 2;

    /**
     * Joint rotates around an axis relative to its parent (hinge joint).
     */
    static final int REVOLUTE = 3;

    /**
     * Joint slides along an axis relative to its parent (slider joint).
     */
    static final int PRISMATIC = 4;

    /**
     * Constructs the root joint of a multibody chain which collides with everything.
     *
     * @param ctx      The context of the app.
     * @param mass     mass of the root joint.
     * @oaran numBones number of child joints in the hierarchy.
     */
    public SXRPhysicsJoint(SXRContext ctx, float mass, int numJoints)
    {
        this(ctx, mass, numJoints,
             (mass > 0) ? SXRCollisionMatrix.DEFAULT_GROUP : SXRCollisionMatrix.STATIC_GROUP);
    }

    /**
     * Constructs the root joint of a multibody chain.
     *
     * @param ctx            The context of the app.
     * @param mass           mass of the root joint.
     * @oaran numJoints      number of child joints in the hierarchy.
     * @param collisionGroup inteeger between 0 and 16 indicating which
     *                       collision group the joint belongs to
     */
    public SXRPhysicsJoint(SXRContext ctx, float mass, int numJoints, int collisionGroup)
    {
        super(ctx, NativePhysicsJoint.ctorRoot(mass, numJoints, 1 << collisionGroup));
        mType = getComponentType();
        mPhysicsContext = SXRPhysicsContext.getInstance();
    }

    /**
     * Constructs the root joint of a multibody chain.
     *
     * @param skel           {@!link SXRSkeleton} driven by this multibody chain.
     * @param mass           mass of the root joint.
     * @oaran numJoints      number of child joints in the hierarchy.
     * @param collisionGroup inteeger between 0 and 16 indicating which
     *                       collision group the joint belongs to
     */
    public SXRPhysicsJoint(SXRSkeleton skel, float mass, int numJoints, int collisionGroup)
    {
        super(skel.getSXRContext(), NativePhysicsJoint.ctorRoot(mass, numJoints, 1 << collisionGroup));
        mSkeleton = skel;
        mType = getComponentType();
        mPhysicsContext = SXRPhysicsContext.getInstance();
    }

    /**
     * Constructs a multibody joint in a chain.
     * <p>
     * This joint is linked to the parent joint in the physics world.
     * @param parent        The parent joint of this one.
     * @param jointType     Type of joint: one of (FIXED, SPHERICAL, REVOLUTE, PRISMATIC, PLANAR)
     * @param jointIndex    0 based bone ID indicating which bone of the skeleton
     *                      this joint belongs to
     * @param mass          mass of this joint
     */
    public SXRPhysicsJoint(SXRPhysicsJoint parent, int jointType, int jointIndex, float mass)
    {
        this(parent, jointType, jointIndex, mass,
             (mass > 0) ? SXRCollisionMatrix.DEFAULT_GROUP : SXRCollisionMatrix.STATIC_GROUP);
    }

    /**
     * Constructs a multibody joint in a chain.
     * <p>
     * This joint is linked to the parent joint in the physics world.
     * This parent should be consistent with the node hierarchy
     * the joints are attached to.
     * @param parent        The parent joint of this one.
     * @param jointType     Type of joint: one of (FIXED, SPHERICAL, REVOLUTE, PRISMATIC, PLANAR)
     * @param jointIndex    0 based bone ID indicating which bone of the skeleton
     *                      this joint belongs to
     * @param mass          mass of this joint
     */
    public SXRPhysicsJoint(SXRPhysicsJoint parent, int jointType, int jointIndex, float mass, int collisionGroup)
    {
        super(parent.getSXRContext(),
              NativePhysicsJoint.ctorLink(parent.getNative(),
                                          jointType, jointIndex, mass, 1 << collisionGroup));
        if (jointIndex < 1)
        {
            throw new IllegalArgumentException("Joint index must be greater than zero");
        }
        mType = getComponentType();
        mSkeleton = parent.mSkeleton;
        mPhysicsContext = SXRPhysicsContext.getInstance();
    }

    /** Used only by {@link SXRPhysicsLoader} */
    SXRPhysicsJoint(SXRContext ctx, long nativeJoint)
    {
        super(ctx, nativeJoint);
        mType = getComponentType();
        mPhysicsContext = SXRPhysicsContext.getInstance();
    }

    static public long getComponentType()
    {
        return NativePhysicsJoint.getComponentType();
    }

    /**
     * Get the name of this joint.
     * <p>
     * If the joint is attached to a scene object, the name
     * of the scene object is returned. Otherwise, the name of the
     * joint last updated by {@link #setName(String)} is returned.
     * If the name has never been set, null is returned.
     * @return name of joint, may be null
     */
    public String getName()
    {
        return NativePhysicsJoint.getName(getNative());
    }

    /**
     * Set the name of this joint.
     * <p>
     * If the joint is attached to a scene object,
     * the name of the scene object is set if it was
     * previously null. If the joint does not have
     * an owner, its local name is updated.
     * If this joint is later attached to a scene
     * object of a different name, the name is not changed.
     */
    public void setName(String name)
    {
        NativePhysicsJoint.setName(getNative(), name);
    }


    /**
     * Establishes how this joint will behave in the simulation.
     *
     * @param type type of simulation desired for the rigid body:
     * <table>
     * <tr><td>DYNAMIC</td><td>Collides with other objects, moved by simulation</td></tr>
     * <tr><td>STATIC</td><td>Collides with other objects, does not move</td></tr>
     * <tr><td>KINEMATIC</td><td>Collides with other objects, moved by application</td></tr>
     * </table>
     */
    public void setSimulationType(final int type)
    {
        mPhysicsContext.runOnPhysicsThread(new Runnable()
        {
            @Override
            public void run()
            {
                NativePhysicsJoint.setSimulationType(getNative(), type);
            }
        });
    }

    /**
     * Queries how this joint will behave in the simulation.
     *
     * @return type of simulation desired for the rigid body
     * <table>
     * <tr><td>DYNAMIC</td><td>Collides with other objects, moved by simulation</td></tr>
     * <tr><td>STATIC</td><td>Collides with other objects, does not move</td></tr>
     * <tr><td>KINEMATIC</td><td>Collides with other objects, moved by application</td></tr>
     * </table>
     */
    public int getSimulationType()
    {
        return NativePhysicsJoint.getSimulationType(getNative());
    }

    public int getJointType() { return NativePhysicsJoint.getJointType(getNative()); }

    /**
     * Set the joint axis for hinge or slider.
     * @param x X direction.
     * @param y Y direction.
     * @param z Z direction.
     */
    public void setAxis(float x, float y, float z)
    {
        NativePhysicsJoint.setAxis(getNative(), x, y, z);
    }

    /**
     * Set the pivot point for this joint.
     * @param x X pivot.
     * @param y Y pivot.
     * @param z Z pivot.
     */
    public void setPivot(float x, float y, float z)
    {
        NativePhysicsJoint.setPivot(getNative(), x, y, z);
    }

    /**
     * Returns the mass of the joint.
     *
     * @return The mass of the joint.
     */
    public float getMass()
    {
        return NativePhysicsJoint.getMass(getNative());
    }

    /**
     * Returns the collision group of this joint..
     *
     * @return The collision group id as an int
     */
    @Override
    public int getCollisionGroup()
    {
        return NativePhysicsJoint.getCollisionGroup(getNative());
    }

    /**
     * Set the linear damping for the multibody system.
     * Linear damping can only be set for the root of the system
     * and applies to all of the joints.
     * @param damping   linear damping value between 0 and 1.
     *                  0 = no damping, 1 = maximum damping
     * @see #setAngularDamping(float)
     * @see #getLinearDamping()
     */
    public void setLinearDamping(float damping)
    {
        if ((damping < 0) || (damping > 1))
        {
            throw new IllegalArgumentException("The damping value must be between 0 and 1");
        }
        if (getJointIndex() > 0)
        {
            throw new UnsupportedOperationException("Linear damping can only be applied to the root of the multibody system");
        }
        NativePhysicsJoint.setLinearDamping(getNative(), damping);
    }

    /**
     * Set the angular damping for the multibody system.
     * Angular damping can only be set for the root of the system
     * and applies to all of the joints.
     * @param damping   linear damping value between 0 and 1.
     *                  0 = no damping, 1 = maximum damping
     * @see #setLinearDamping(float)
     * @see #getAngularDamping()
     */
    public void setAngularDamping(float damping)
    {
        if ((damping < 0) || (damping > 1))
        {
            throw new IllegalArgumentException("The damping value must be between 0 and 1");
        }
        if (getJointIndex() > 0)
        {
            throw new UnsupportedOperationException("Linear damping can only be applied to the root of the multibody system");
        }
        NativePhysicsJoint.setAngularDamping(getNative(), damping);
    }

    /**
     * Returns the linear damping for the multibody system.
     *
     * @return linear damping value between 0 and 1
     * @see #setLinearDamping(float)
     */
    public float getLinearDamping()
    {
        return NativePhysicsJoint.getLinearDamping(getNative());
    }

    /**
     * Returns the linear damping for the multibody system.
     *
     * @return linear damping value between 0 and 1
     * @see #setAngularDamping(float)
     */
    public float getAngularDamping()
    {
        return NativePhysicsJoint.getAngularDamping(getNative());
    }


    /**
     * Set the maximum applied impulse for the multibody system.
     * This factor can only be set for the root of the system
     * and applies to all of the joints.
     * @param v   non-negative maximum applied impulse
     * @see #setLinearDamping(float)
     * @see #getAngularDamping()
     */
    public void setMaxAppliedImpulse(float v)
    {
        if (v < 0)
        {
            throw new IllegalArgumentException("The maximum applied impulse value cannot be negative");
        }
        if (getJointIndex() > 0)
        {
            throw new UnsupportedOperationException("Maximum applied impulse can only be set on the root of the multibody system");
        }
        NativePhysicsJoint.setMaxAppliedImpulse(getNative(), v);
    }

    /**
     * Set the maximum applied impulse for the multibody system.
     * This factor can only be set for the root of the system
     * and applies to all of the joints.
     * @param v   non-negative maximum applied impulse
     * @see #getMaxCoordVelocity()
     */
    public void setMaxCoordVelocity(float v)
    {
        if (v < 0)
        {
            throw new IllegalArgumentException("The maximum coordinate velocity cannot be negative");
        }
        if (getJointIndex() > 0)
        {
            throw new UnsupportedOperationException("Maximum coordinate velocity can only be set on the root of the multibody system");
        }
        NativePhysicsJoint.setMaxCoordVelocity(getNative(), v);
    }

    /**
     * Returns the maximum applied impluse for the multibody system.
     *
     * @return maximum applied impulse
     * @see #setMaxAppliedImpulse(float)
     */
    public float getMaxAppliedImpulse()
    {
        return NativePhysicsJoint.getMaxAppliedImpulse(getNative());
    }

    /**
     * Returns the maximum applied impluse for the multibody system.
     *
     * @return maximum applied impulse
     * @see #setMaxCoordVelocity(float)
     */
    public float getMaxCoordVelocity()
    {
        return NativePhysicsJoint.getMaxCoordVelocity(getNative());
    }

    /**
     * Set the friction for this joint.
     * @param friction  friction value
     * @see #getFriction()
     */
    public void setFriction(float friction)
    {
        NativePhysicsJoint.setFriction(getNative(), friction);
    }

    /**
     * Get the friction for this joint.
     * @return friction value
     * @see #setFriction(float)
     */
    public float getFriction()
    {
        return NativePhysicsJoint.getFriction(getNative());
    }


    /**
     * Apply a torque vector [X, Y, Z] to this {@linkplain SXRPhysicsJoint joint}
     *
     * @param x factor on the 'X' axis.
     * @param y factor on the 'Y' axis.
     * @param z factor on the 'Z' axis.
     */
    public void applyTorque(final float x, final float y, final float z)
    {
        SXRPhysicsContext.getInstance().runOnPhysicsThread(new Runnable() {
            @Override
            public void run() {
                NativePhysicsJoint.applyTorque(getNative(), x, y, z);
            }
        });
    }

    /**
     * Apply a force vector [X, Y, Z] to this {@linkplain SXRPhysicsJoint joint}
     *
     * @param x factor on the 'X' axis.
     * @param y factor on the 'Y' axis.
     * @param z factor on the 'Z' axis.
     */
    public void applyCentralForce(final float x, final float y, final float z)
    {
        SXRPhysicsContext.getInstance().runOnPhysicsThread(new Runnable() {
            @Override
            public void run() {
                NativePhysicsJoint.applyCentralForce(getNative(), x, y, z);
            }
        });
    }

    /**
     * Get the total number of joints in the multibody system.
     * @return number of joints, will always be positive.
     */
    public int getNumJoints()
    {
        return NativePhysicsJoint.getNumJoints(getNative());
    }


    /**
     * Apply a torque to a single DOF joint {@linkplain SXRPhysicsJoint joint}
     *
     * @param t torque on the joint
     */
    public void applyTorque(final float t)
    {
        SXRPhysicsContext.getInstance().runOnPhysicsThread(new Runnable() {
            @Override
            public void run() {
                NativePhysicsJoint.applyTorque(getNative(), t, 0, 0);
            }
        });
    }

    /**
     * Returns the 0 based index of this joint.
     */
    public int getJointIndex() { return NativePhysicsJoint.getJointIndex(getNative()) + 1; }

    public float[] getAxis() { return NativePhysicsJoint.getAxis(getNative()); }

    /**
     * Returns the transform used to position and orient the collider
     * with respect to the joint.
     *
     * @return float array with 16 elements containing a 4x4 matrix
     */
    public float[] getColliderTransform()
    {
        return NativePhysicsJoint.getColliderTransform(getNative());
    }

    /**
     * Sets the transform used to position and orient the collider
     * with respect to the joint.
     *
     * @param matrix float array with 16 elements containing a 4x4 matrix
     */
    public void setColliderTransform(final float[] matrix)
    {
        mPhysicsContext.runOnPhysicsThread(new Runnable()
        {
            public void run()
            {
                NativePhysicsJoint.setColliderTransform(getNative(), matrix);
            }
        });
    }

    public void removeJointAt(int jointIndex)
    {
        SXRNode owner = getOwnerObject();

        if (owner != null)
        {
            owner.detachComponent(SXRPhysicsJoint.getComponentType());
        }
        NativePhysicsJoint.removeJointAt(getNative(), jointIndex);
    }

    public int addJoint(SXRPhysicsJoint childJoint)
    {
        SXRNode childOwner = childJoint.getOwnerObject();

        if (childOwner != null)
        {
            childOwner.detachComponent(SXRPhysicsJoint.getComponentType());
        }
        childJoint.mSkeleton = mSkeleton;
        return NativePhysicsJoint.addJoint(getNative(), childJoint.getNative());
    }

    public SXRSkeleton getSkeleton()
    {
        if (mSkeleton != null)
        {
            return mSkeleton;
        }
        mSkeleton = (SXRSkeleton) getComponent(SXRSkeleton.getComponentType());
        if (mSkeleton != null)
        {
            return mSkeleton;
        }
        long nativeSkeleton = NativePhysicsJoint.getSkeleton(getNative());
        if (nativeSkeleton != 0)
        {
            mSkeleton = new SXRSkeleton(getSXRContext(), nativeSkeleton);
        }
        return mSkeleton;
    }

    public void copy(SXRPhysicsJoint srcJoint)
    {
        SXRNode srcOwner = srcJoint.getOwnerObject();
        SXRNode destOwner = getOwnerObject();
        SXRCollider srcCollider = null;

        if (srcOwner != null)
        {
            srcCollider = (SXRCollider) srcOwner.getComponent(SXRCollider.getComponentType());
            srcOwner.detachComponent(SXRCollider.getComponentType());
        }
        if (destOwner != null)
        {
            destOwner.detachComponent(SXRCollider.getComponentType());
            destOwner.attachCollider(srcCollider);
        }
        NativePhysicsJoint.copy(getNative(), srcJoint.getNative());
    }

    @Override
    public void onAttach(SXRNode newOwner)
    {
        if (newOwner.getCollider() == null)
        {
            throw new UnsupportedOperationException("You must have a collider attached to the node before attaching the joint component");
        }
        super.onAttach(newOwner);
    }

    @Override
    public void onDisable()
    {
        super.onDisable();

        if (getJointIndex() == 0)
        {
            removeFromWorld(getWorld());
        }
    }

    @Override
    public void sync(int options)
    {
        NativePhysicsJoint.sync(getNative(), options);
    }

    @Override
    protected void addToWorld(SXRPhysicsContent world)
    {
        if (world != null)
        {
            if (!world.isMultiBody())
            {
                throw new IllegalArgumentException("SXRPhysicsJoint can only be added if multibody is enabled on the physics world");
            }
            world.addBody(this);
        }
    }

    @Override
    protected void removeFromWorld(SXRPhysicsContent world)
    {
        if (world != null)
        {
            world.removeBody(this);
        }
    }
}

class NativePhysicsJoint
{
    static native long ctorRoot(float mass, int numBones, int collisionGroup);
    static native long ctorLink(long parent_joint, int jointType, int boneid, float mass, int collisionGroup);
    static native long getComponentType();
    static native void copy(long destjoint, long srcjoint);

    static native float   getMass(long joint);
    static native int     getJointIndex(long joint);
    static native float   getLinearDamping(long joint);
    static native float   getAngularDamping(long joint);
    static native float   getMaxAppliedImpulse(long joint);
    static native float   getMaxCoordVelocity(long joint);
    static native float   getFriction(long joint);
    static native long    getSkeleton(long joint);
    static native String  getName(long joint);
    static native int     getNumJoints(long joint);
    static native int     getJointType(long joint);
    static native int     getCollisionGroup(long joint);
    static native int     getSimulationType(long jjoint);
    static native float[] getAxis(long jjoint);
    static native float[] getColliderTransform(long jjoint);

    static native void setFriction(long joint, float friction);
    static native void setAxis(long joint, float x, float y, float z);
    static native void setPivot(long joint, float x, float y, float z);
    static native void setName(long joint, String name);
    static native void setLinearDamping(long joint, float d);
    static native void setAngularDamping(long joint, float d);
    static native void setMaxAppliedImpulse(long joint, float d);
    static native void setMaxCoordVelocity(long joint, float d);
    static native void setSimulationType(long jjoint, int jtype);
    static native void setColliderTransform(long jjoint, float[] matrix);

    static native void applyTorque(long joint, float x, float y, float z);
    static native void applyCentralForce(long joint, float x, float y, float z);

    static native void removeJointAt(long joint, int index);
    static native int  addJoint(long rootjoint, long newjoint);
    static native void sync(long joint, int options);
}
