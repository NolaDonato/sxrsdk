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
import com.samsungxr.SXRRenderData;
import com.samsungxr.SXRNode;
import com.samsungxr.animation.SXRSkeleton;

import org.joml.Vector3f;

import java.lang.annotation.Native;

/**
 * Represents a rigid body with physical properties that can
 * collide with other objects.
 * <p>
 * A rigid body can be static, kinematic or dynamic.
 * Static bodies don't move at all, kinematic bodies are moved by
 * animation in the application and dynamic bodies are moved by
 * the physics engine.
 * <p>
 * Rigid bodies also have mass and respond to physical forces.
 * To participate in collisions, a rigid body must have a
 * {@link SXRCollider} component attached to its owner
 * which describes the shape of the rigid body.
 * <p>
 * By default a rigid body is static with infinity mass, value 0, and does not move under simulation.
 * A dynamic body with a mass defined is fully simulated.
 * <p>
 * The rigid body component is attached to a {@linkplain com.samsungxr.SXRNode node}
 * and uses the transform of its owner object, updating it if the body is dynamic.
 * Before attaching a rigid body to a node, make sure the node has the proper
 * position and orientation. You cannot attach a rigid body to a node
 * unless it is in the scene and has a collider component.
 * @see SXRNode
 * @see SXRCollider
 * @see SXRWorld
 */
public class SXRRigidBody extends SXRPhysicsCollidable
{
    public static final int DYNAMIC  = 0;
    public static final int STATIC = 1;
    public static final int KINEMATIC = 2;

    static {
        System.loadLibrary("sxr-physics");
    }

    private final SXRPhysicsContext mPhysicsContext;

    /**
     * Constructs a new static rigid body with zero mass.
     *
     * @param ctx The context of the app.
     */
    public SXRRigidBody(SXRContext ctx) {
        this(ctx, 0.0f);
    }


    /**
     * Constructs a new rigid body with the given mass in
     * collision group 0.
     * <p>
     * If the input mass is zero, the rigid body will
     * be designated static and cannot move.
     * Otherwise, it is marked as dynamic and will
     * be fully simulated.
     * </p>
     * To make a kinematic rigid body, call the
     * {@link #setSimulationType(int)} funcion.
     * @param ctx   The context of the app.
     * @param mass  The mass of this rigid body.
     */
    public SXRRigidBody(SXRContext ctx, float mass)
    {
        super(ctx, NativeRigidBody.ctor(mass,
                (mass > 0) ? (1 << SXRCollisionMatrix.DEFAULT_GROUP) : (1 << SXRCollisionMatrix.STATIC_GROUP)));
        mType = getComponentType();
        mPhysicsContext = SXRPhysicsContext.getInstance();
    }

    /**
     * Constructs a new rigid body with the given mass belonging to
     * the specified collision group.
     * <p>
     * If the input mass is zero, the rigid body will
     * be designated static and cannot move.
     * Otherwise, it is marked as dynamic and will
     * be fully simulated.
     * </p>
     * To make a kinematic rigid body, call the
     * {@link #setSimulationType(int)} funciont.
     *
     * @param ctx The context of the app.
     * @param mass The mass of this rigid body.
     * @param collisionGroup The id of the collision's group that this rigid body belongs to
     *                       in the {@link SXRCollisionMatrix}. The rigid body collides with
     *                       everyone if {#collisionGroup} is out of the range 0...15.
     */
    public SXRRigidBody(SXRContext ctx, float mass, int collisionGroup)
    {
        super(ctx, NativeRigidBody.ctor(mass, 1 << collisionGroup));
        mType = getComponentType();
        mPhysicsContext = SXRPhysicsContext.getInstance();
    }

    /** Used only by {@link SXRPhysicsLoader} */
    SXRRigidBody(SXRContext ctx, long nativeRigidBody)
    {
        super(ctx, nativeRigidBody);
        mType = getComponentType();
        mPhysicsContext = SXRPhysicsContext.getInstance();
    }

    static public long getComponentType() {
        return NativeRigidBody.getComponentType();
    }

    /**
     * Get the name of this rigid body.
     * <p>
     * If the rigid body is attached to a scene object, the name
     * of the scene object is returned. Otherwise, the name of the
     * rigid body last updated by {@link #setName(String)} is returned.
     * If the name has never been set, null is returned.
     * @return name of rigid body, may be null
     */
    public String getName()
    {
        return NativeRigidBody.getName(getNative());
    }

    /**
     * Set the name of this rigid body.
     * <p>
     * If the rigid body is attached to a scene object,
     * the name of the scene object is set if it was
     * previously null. If the rigid body does not have
     * an owner, its local name is updated.
     * If this rigid body is later attached to a scene
     * object of a different name, the name is not changed.
     */
    public void setName(String name)
    {
        NativeRigidBody.setName(getNative(), name);
    }

    public void setScale(final float x, final float y, final float z)
    {
        mPhysicsContext.runOnPhysicsThread(new Runnable()
        {
            @Override
            public void run()
            {
                NativeRigidBody.setScale(getNative(), x, y, z);
            }
        });
    }

    public void setScale(Vector3f v)
    {
        setScale(v.x, v.y, v.z);
    }

    public void getScale(Vector3f v)
    {
        float[] scale = getScale();
        v.x = scale[0];
        v.y = scale[1];
        v.z = scale[2];
    }

    public float[] getScale()
    {
        return NativeRigidBody.getScale(getNative());
    }

    /**
     * Establishes how this rigid body will behave in the simulation.
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
                NativeRigidBody.setSimulationType(getNative(), type);
            }
        });
    }

    /**
     * Queries how this rigid body will behave in the simulation.
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
       return NativeRigidBody.getSimulationType(getNative());
    }

    /**
     * Returns the mass of the body.
     *
     * @return The mass of the body.
     */
    public float getMass() {
        return NativeRigidBody.getMass(getNative());
    }

    /**
     * Apply a central force vector [X, Y, Z] to this {@linkplain SXRRigidBody rigid body}
     *
     * @param x factor on the 'X' axis.
     * @param y factor on the 'Y' axis.
     * @param z factor on the 'Z' axis.
     */
    public void applyCentralForce(final float x, final float y, final float z)
    {
        mPhysicsContext.runOnPhysicsThread(new Runnable() {
            @Override
           public void run()
           {
               NativeRigidBody.applyCentralForce(getNative(), x, y, z);
           }
        });
    }

    /**
     * Apply a force vector [X, Y, Z] to this {@linkplain SXRRigidBody rigid body}
     * on relative position
     *
     * @param forceX force on the x-axis.
     * @param forceY force on the y-axis.
     * @param forceZ force on the z-axis.
     * @param relX relative position on x-axis to apply the force.
     * @param relY relative position on y-axis to apply the force.
     * @param relZ relative position on z-axis to apply the force.
     */
    public void applyForce(final float forceX, final float forceY, final float forceZ,
                           final float relX, final float relY, final float relZ)
    {
        mPhysicsContext.runOnPhysicsThread(new Runnable() {
            @Override
            public void run() {
                NativeRigidBody.applyForce(getNative(), forceX, forceY, forceZ,
                                           relX, relY, relZ);
            }
        });
    }

    /**
     * Apply a central vector vector [X, Y, Z] to this {@linkplain SXRRigidBody rigid body}
     *
     * @param x impulse factor on the 'X' axis.
     * @param y impulse factor on the 'Y' axis.
     * @param z impulse factor on the 'Z' axis.
     */
    public void applyCentralImpulse(final float x, final float y, final float z)
    {
        mPhysicsContext.runOnPhysicsThread(new Runnable() {
            @Override
            public void run() {
                NativeRigidBody.applyCentralImpulse(getNative(), x, y, z);
            }
        });
    }

    /**
     * Apply a impulse vector [X, Y, Z] to this {@linkplain SXRRigidBody rigid body}
     * on relative position
     *
     * @param impulseX impulse on the x-axis.
     * @param impulseY impulse on the y-axis.
     * @param impulseZ impulse on the z-axis.
     * @param relX relative position on x-axis to apply the force.
     * @param relY relative position on y-axis to apply the force.
     * @param relZ relative position on z-axis to apply the force.
     */
    public void applyImpulse(final float impulseX, final float impulseY, final float impulseZ,
                           final float relX, final float relY, final float relZ)
    {
        mPhysicsContext.runOnPhysicsThread(new Runnable()
        {
            @Override
            public void run()
            {
                NativeRigidBody.applyImpulse(getNative(), impulseX, impulseY, impulseZ,
                                             relX, relY, relZ);
            }
        });
    }

    /**
     * Apply a torque vector [X, Y, Z] to this {@linkplain SXRRigidBody rigid body}
     *
     * @param x factor on the 'X' axis.
     * @param y factor on the 'Y' axis.
     * @param z factor on the 'Z' axis.
     */
    public void applyTorque(final float x, final float y, final float z)
    {
        mPhysicsContext.runOnPhysicsThread(new Runnable()
        {
            @Override
            public void run()
            {
                NativeRigidBody.applyTorque(getNative(), x, y, z);
            }
        });
    }

    /**
     * Apply a torque impulse [X, Y, Z] to this {@linkplain SXRRigidBody rigid body}
     *
     * @param x impulse factor on the 'X' axis.
     * @param y impulse factor on the 'Y' axis.
     * @param z impulse factor on the 'Z' axis.
     */
    public void applyTorqueImpulse(final float x, final float y, final float z)
    {
        mPhysicsContext.runOnPhysicsThread(new Runnable()
        {
            @Override
            public void run()
            {
                NativeRigidBody.applyTorqueImpulse(getNative(), x, y, z);
            }
        });
    }

    /**
     * Sets a particular acceleration vector [X, Y, Z] on this {@linkplain SXRRigidBody rigid body}
     *
     * @param x factor on the 'X' axis.
     * @param y factor on the 'Y' axis.
     * @param z factor on the 'Z' axis.
     */
    public void setGravity(final float x, final float y, final float z)
    {
        mPhysicsContext.runOnPhysicsThread(new Runnable()
        {
            @Override
            public void run()
            {
                NativeRigidBody.setGravity(getNative(), x, y, z);
            }
        });
    }

    /**
     * Sets linear and angular damping on this {@linkplain SXRRigidBody rigid body}
     *
     * @param linear factor on how much the rigid body resists translation.
     * @param angular factor on how much the rigid body resists rotation.
     */
    public void setDamping(final float linear, final float angular)
    {
        mPhysicsContext.runOnPhysicsThread(new Runnable()
        {
            @Override
            public void run()
            {
                NativeRigidBody.setDamping(getNative(), linear, angular);
            }
        });
    }

    /**
     * Sets a linear velocity [X, Y, Z] on this {@linkplain SXRRigidBody rigid body}
     *
     * @param x factor on the 'X' axis.
     * @param y factor on the 'Y' axis.
     * @param z factor on the 'Z' axis.
     */
    public void setLinearVelocity(final float x, final float y, final float z)
    {
        mPhysicsContext.runOnPhysicsThread(new Runnable()
        {
            @Override
            public void run()
            {
                NativeRigidBody.setLinearVelocity(getNative(), x, y, z);
            }
        });
    }

    /**
     * Sets an angular velocity [X, Y, Z] on this {@linkplain SXRRigidBody rigid body}
     *
     * @param x factor on the 'X' axis.
     * @param y factor on the 'Y' axis.
     * @param z factor on the 'Z' axis.
     */
    public void setAngularVelocity(final float x, final float y, final float z)
    {
        mPhysicsContext.runOnPhysicsThread(new Runnable()
        {
            @Override
            public void run()
            {
                NativeRigidBody.setAngularVelocity(getNative(), x, y, z);
            }
        });
    }

    /**
     * Sets an angular factor [X, Y, Z] that influences torque on this {@linkplain SXRRigidBody rigid body}
     *
     * @param x factor on the 'X' axis.
     * @param y factor on the 'Y' axis.
     * @param z factor on the 'Z' axis.
     */
    public void setAngularFactor(final float x, final float y, final float z)
    {
        mPhysicsContext.runOnPhysicsThread(new Runnable()
        {
            @Override
            public void run()
            {
                NativeRigidBody.setAngularFactor(getNative(), x, y, z);
            }
        });
    }

    /**
     * Sets an linear factor [X, Y, Z] that influences forces acting on this {@linkplain SXRRigidBody rigid body}
     *
     * @param x factor on the 'X' axis.
     * @param y factor on the 'Y' axis.
     * @param z factor on the 'Z' axis.
     */
    public void setLinearFactor(final float x, final float y, final float z)
    {
        mPhysicsContext.runOnPhysicsThread(new Runnable()
        {
            @Override
            public void run()
            {
                NativeRigidBody.setLinearFactor(getNative(), x, y, z);
            }
        });
    }

    /**
     * Sets SleepingTresholds that, when reached, influence the deactivation of this {@linkplain SXRRigidBody rigid body}
     *
     * @param linear factor for the linearVelocity
     * @param angular factor for the angularVelocity
     */
    public void setSleepingThresholds(final float linear, final float angular)
    {
        mPhysicsContext.runOnPhysicsThread(new Runnable()
        {
            @Override
            public void run()
            {
                NativeRigidBody.setSleepingThresholds(getNative(), linear, angular);
            }
        });
    }

    /**
     * Set a {@linkplain SXRRigidBody rigid body} to be ignored (true) or not (false)
     *
     * @param collisionObject rigidbody object on the collision check
     * @param ignore boolean to indicate if the specified object will be ignored or not
     */
    public void setIgnoreCollisionCheck(final SXRRigidBody collisionObject, final boolean ignore)
    {
        mPhysicsContext.runOnPhysicsThread(new Runnable()
        {
            @Override
            public void run()
            {
                NativeRigidBody.setIgnoreCollisionCheck(getNative(), collisionObject.getNative(), ignore);
            }
        });
    }

    /**
     * Set the restitution factor of this {@linkplain SXRRigidBody rigid body}
     *
     * @param n the restitution factor
     */
    public void setRestitution(final float n)
    {
        mPhysicsContext.runOnPhysicsThread(new Runnable()
        {
            @Override
            public void run()
            {
                NativeRigidBody.setRestitution(getNative(), n);
            }
        });
    }

    /**
     * Set the friction factor of this {@linkplain SXRRigidBody rigid body}
     *
     * @param n the friction factor
     */
    public void setFriction(final float n)
    {
        mPhysicsContext.runOnPhysicsThread(new Runnable()
        {
            @Override
            public void run()
            {
                NativeRigidBody.setFriction(getNative(), n);
            }
        });
    }


    /**
     * Set the continuous collision detection motion threshold factor of this {@linkplain SXRRigidBody rigid body}
     *
     * @param n the continuous collision detection motion threshold factor
     */
    public void setCcdMotionThreshold(final float n)
    {
        mPhysicsContext.runOnPhysicsThread(new Runnable()
        {
            @Override
            public void run()
            {
                NativeRigidBody.setCcdMotionThreshold(getNative(), n);
            }
        });
    }


    /**
     * Set the sphere to continuous collision detection of this {@linkplain SXRRigidBody rigid body}
     *
     * @param n Radius of sphere to continuous collision detection.
     */
    public void setCcdSweptSphereRadius(final float n)
    {
        mPhysicsContext.runOnPhysicsThread(new Runnable()
        {
            @Override
            public void run()
            {
                NativeRigidBody.setCcdSweptSphereRadius(getNative(), n);
            }
        });
    }

    /**
     * Set the  contact processing threshold factor of this {@linkplain SXRRigidBody rigid body}
     *
     * @param n the contact processing threshold factor
     */
    public void setContactProcessingThreshold(final float n)
    {
        mPhysicsContext.runOnPhysicsThread(new Runnable()
        {
            @Override
            public void run()
            {
                NativeRigidBody.setContactProcessingThreshold(getNative(), n);
            }
        });
    }

    /**
     * Returns the gravity acceleration float array [x,y,z] on this {@linkplain SXRRigidBody rigid body}.
     *
     * @return The gravity acceleration vector as a float array
     */
    public float[] getGravity() {
        return NativeRigidBody.getGravity(getNative());
    }

    /**
     * Returns the linear velocity float array [x,y,z] on this {@linkplain SXRRigidBody rigid body}.
     *
     * @return The linear velocity vector as a float array
     */
    public float[] getLinearVelocity() {
        return NativeRigidBody.getLinearVelocity(getNative());
    }

    /**
     * Returns the angular velocity float array [x,y,z] on this {@linkplain SXRRigidBody rigid body}.
     *
     * @return The angular velocity vector as a float array
     */
    public float[] getAngularVelocity() {
        return NativeRigidBody.getAngularVelocity(getNative());
    }

    /**
     * Returns the angular factor float array [x,y,z] on this {@linkplain SXRRigidBody rigid body}.
     *
     * @return The angular factor vector as a float array
     */
    public float[] getAngularFactor() {
        return NativeRigidBody.getAngularFactor(getNative());
    }

    /**
     * Returns the linear factor float array [x,y,z] on this {@linkplain SXRRigidBody rigid body}.
     *
     * @return The linear factor vector as a float array
     */
    public float[] getLinearFactor() {
        return NativeRigidBody.getLinearFactor(getNative());
    }

    /**
     * Returns the damping factors [angular,linear] on this {@linkplain SXRRigidBody rigid body}.
     *
     * @return The damping factors as a float array
     */
    public float[] getDamping() {
        return NativeRigidBody.getDamping(getNative());
    }

    /**
     * Returns the friction factor on this {@linkplain SXRRigidBody rigid body}.
     *
     * @return The friction factor scalar as a float
     */
    public float getFriction() {
        return NativeRigidBody.getFriction(getNative());
    }

    /**
     * Returns the restitution factor on this {@linkplain SXRRigidBody rigid body}.
     *
     * @return The restitution factor scalar as a float
     */
    public float getRestitution() {
        return NativeRigidBody.getRestitution(getNative());
    }

    /**
     * Returns the continuous collision detection motion threshold factor on this {@linkplain SXRRigidBody rigid body}.
     *
     * @return The continuous collision detection motion threshold factor scalar as a float
     */
    public float getCcdMotionThreshold()
    {
        return NativeRigidBody.getCcdMotionThreshold(getNative());
    }

    /**
     * Returns the contact processing threshold factor for this {@linkplain SXRRigidBody rigid body}.
     *
     * @return The contact processing threshold factor scalar as a float
     */
    public float getContactProcessingThreshold()
    {
        return NativeRigidBody.getContactProcessingThreshold(getNative());
    }

    /**
     * Returns the radius of sphere used to collision detection on this {@linkplain SXRRigidBody rigid body}.
     *
     * @return The radius of sphere to continuous collision detection.
     */
    public float getCcdSweptSphereRadius()
    {
        return NativeRigidBody.getCcdSweptSphereRadius(getNative());
    }

    /**
     * Returns the collision group of this {@linkplain SXRRigidBody rigid body}.
     *
     * @return The collision group id as an int between 9 and 15.
     */
    @Override
    public int getCollisionGroup()
    {
        return NativeRigidBody.getCollisionGroup(getNative());
    }

    @Override
    public void sync(int options)
    {
        NativeRigidBody.sync(getNative(), options);
    }

    @Override
    public void onAttach(SXRNode newOwner)
    {
        if (newOwner.getCollider() == null)
        {
            throw new UnsupportedOperationException("You must have a collider attached to the node before attaching the rigid body");
        }
        super.onAttach(newOwner);
    }

    @Override
    protected void addToWorld(SXRPhysicsContent world)
    {
        if (world != null)
        {
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

class NativeRigidBody
{
    static native long ctor(float mass, int collisionGroup);
    static native long getComponentType();

    static native float   getMass(long jrigid_body);
    static native String  getName(long jrigid_body);
    static native float[] getGravity(long jrigid_body);
    static native float[] getLinearVelocity(long jrigid_body);
    static native float[] getAngularVelocity(long jrigid_body);
    static native float[] getAngularFactor(long jrigid_body);
    static native float[] getLinearFactor(long jrigid_body);
    static native float[] getDamping(long jrigid_body);
    static native float   getFriction(long jrigid_body);
    static native float   getRestitution(long jrigid_body);
    static native float   getCcdMotionThreshold(long jrigid_body);
    static native float   getCcdSweptSphereRadius(long jrigid_body);
    static native float   getContactProcessingThreshold(long jrigid_body);
    static native int     getSimulationType(long jrigid_body);
    static native float[] getScale(long jrigid_body);
    static native int     getCollisionGroup(long jrigid_body);

    static native void setGravity(long jrigid_body, float x, float y, float z);
    static native void setDamping(long jrigid_body, float linear, float angular);
    static native void setLinearVelocity(long jrigid_body, float x, float y, float z);
    static native void setAngularVelocity(long jrigid_body, float x, float y, float z);
    static native void setAngularFactor(long jrigid_body, float x, float y, float z);
    static native void setLinearFactor(long jrigid_body, float x, float y, float z);
    static native void setFriction(long jrigid_body, float n);
    static native void setRestitution(long jrigid_body, float n);
    static native void setSleepingThresholds(long jrigid_body, float linear, float angular);
    static native void setCcdMotionThreshold(long jrigid_body, float n);
    static native void setCcdSweptSphereRadius(long jrigid_body, float n);
    static native void setContactProcessingThreshold(long jrigid_body, float n);
    static native void setIgnoreCollisionCheck(long jrigid_body, long jcollision_object, boolean ignore);
    static native void setScale(long jrigid_body, float x, float y, float z);
    static native void setName(long jrigid_body, String name);
    static native void setSimulationType(long jrigid_body, int jtype);

    static native void applyCentralForce(long jrigid_body, float x, float y, float z);
    static native void applyForce(long jrigid_body, float force_x, float force_y, float force_z,
                                  float rel_pos_x, float rel_pos_y, float rel_pos_z);
    static native void applyCentralImpulse(long jrigid_body, float x, float y, float z);
    static native void applyImpulse(long jrigid_body, float impulse_x, float impulse_y, float impulse_z,
                                  float rel_pos_x, float rel_pos_y, float rel_pos_z);
    static native void applyTorque(long jrigid_body, float x, float y, float z);
    static native void applyTorqueImpulse(long jrigid_body, float x, float y, float z);
    static native void sync(long jrigid_body, int options);
}
