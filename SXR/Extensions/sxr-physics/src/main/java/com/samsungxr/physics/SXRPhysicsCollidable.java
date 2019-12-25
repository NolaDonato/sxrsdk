package com.samsungxr.physics;

import com.samsungxr.SXRComponent;
import com.samsungxr.SXRContext;
import com.samsungxr.SXRNode;

import org.joml.Vector3f;

import java.util.List;

/**
 * Represents a physics component that participate in collisions.
 * <p>
 * This includes rigid bodies and joints but not constraints.
 * @see SXRRigidBody
 * @see SXRPhysicsJoint
 */
abstract public class SXRPhysicsCollidable extends SXRPhysicsWorldObject
{
    public static final int SYNC_COLLISION_SHAPE = 1;
    public static final int SYNC_TRANSFORM = 2;
    public static final int SYNC_PROPERTIES = 4;
    public static final int SYNC_ALL = SYNC_COLLISION_SHAPE | SYNC_TRANSFORM | SYNC_PROPERTIES;

    protected SXRPhysicsCollidable(SXRContext gvrContext, long nativePointer)
    {
        super(gvrContext, nativePointer);
    }

    protected SXRPhysicsCollidable(SXRContext gvrContext, long nativePointer, List<NativeCleanupHandler> cleanupHandlers)
    {
        super(gvrContext, nativePointer, cleanupHandlers);
    }

    abstract public int getCollisionGroup();
    abstract public String getName();
    abstract public void getScale(Vector3f v);
    abstract public float[] getScale();
    abstract public int getSimulationType();

    abstract public void setName(String s);
    abstract public void setScale(float x, float y, float z);
    abstract public void setScale(Vector3f v);
    abstract public void setSimulationType(int t);

    abstract public void sync(int syncOptions);
}
