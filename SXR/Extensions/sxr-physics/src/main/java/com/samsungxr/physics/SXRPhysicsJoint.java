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

import com.samsungxr.SXRComponent;
import com.samsungxr.SXRContext;
import com.samsungxr.SXRNode;

/**
 * Represents a joint in a multibody chain like a ragdoll.
 * <p>
 */
public class SXRPhysicsJoint extends SXRPhysicsWorldObject
{
    /**
     * Constructs new instance to simulate a rigid body in {@link SXRWorld}.
     *
     * @param gvrContext The context of the app.
     * @param boneID   0 based bone ID indicating which bone of the skeleton
     *                 this joint belongs to
     */
    public SXRPhysicsJoint(SXRContext gvrContext, float baseMass, int boneID)
    {
        super(gvrContext, NativePhysicsJoint.ctor(baseMass, boneID));
    }


    /** Used only by {@link SXRPhysicsLoader} */
    SXRPhysicsJoint(SXRContext gvrContext, long nativeJoint)
    {
        super(gvrContext, nativeJoint);
    }

    static public long getComponentType() {
        return NativePhysicsJoint.getComponentType();
    }

    /**
     * Returns the {@linkplain SXRWorld physics world} of this {@linkplain SXRRigidBody rigid body}.
     *
     * @return The physics world of this {@link SXRRigidBody}
     */
    public SXRWorld getWorld() {
        return getWorld(getOwnerObject());
    }

    /**
     * Returns the {@linkplain SXRWorld physics world} of the {@linkplain com.samsungxr.SXRScene scene}.
     *
     * @param owner Owner of the {@link SXRRigidBody}
     * @return Returns the {@link SXRWorld} of the scene.
     */
    private static SXRWorld getWorld(SXRNode owner) {
        return getWorldFromAscendant(owner);
    }

    /**
     * Looks for {@link SXRWorld} component in the ascendants of the scene.
     *
     * @param worldOwner Scene object to search for a physics world in the scene.
     * @return Physics world from the scene.
     */
    private static SXRWorld getWorldFromAscendant(SXRNode worldOwner) {
        SXRComponent world = null;

        while (worldOwner != null && world == null) {
            world = worldOwner.getComponent(SXRWorld.getComponentType());
            worldOwner = worldOwner.getParent();
        }

        return (SXRWorld) world;
    }

    /**
     * Returns the mass of the body.
     *
     * @return The mass of the body.
     */
    public float getMass() {
        return NativePhysicsJoint.getMass(getNative());
    }

    /**
     * Returns the bone ID of this joint.
     */
    public int getBoneID() { return NativePhysicsJoint.getBoneID(getNative()); }

    @Override
    public void onAttach(SXRNode newOwner)
    {
        if (newOwner.getCollider() == null)
        {
            throw new UnsupportedOperationException("You must have a collider attached to the node before attaching the multi body component");
        }
        super.onAttach(newOwner);
    }

    @Override
    protected void addToWorld(SXRWorld world) {
        if (world != null) {
            world.addBody(this);
        }
    }

    @Override
    protected void removeFromWorld(SXRWorld world) {
        if (world != null) {
            world.removeBody(this);
        }
    }
}

class NativePhysicsJoint
{
    static native long ctor(float mass, int link);

    static native long getComponentType();

    static native float getMass(long joint);

    static native int getBoneID(long joint);
}
