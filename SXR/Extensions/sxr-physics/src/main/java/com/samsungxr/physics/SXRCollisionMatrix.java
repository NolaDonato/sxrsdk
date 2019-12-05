package com.samsungxr.physics;

/**
 * The Collision Matrix defines the collision between groups of {@link SXRPhysicsCollidable} objects.
 * <p>
 * A collision group is a number between 0 and 15 that is a index into the collision matrix.
 * A collidable (rigid body or joint) that does not belong to a collision group collides with every
 * physics object in the {@link SXRWorld}. By default a collidable that belongs to a group
 * collides only with other objects of the same collision group.
 */
public class SXRCollisionMatrix
{

    /*
     * Enable collision with dynamic bodies.
     */
   public static final int DEFAULT_GROUP = 0;

    /*
     * Enable collision with static bodies.
     */
    public static final int STATIC_GROUP = 1;

    /*
     * Enable collision with kinematic bodies.
     */
    public static final int KINEMATIC_GROUP = 2;

    /*
     * Enable collision with debris.
     */
    public static final int DEBRIS_GROUP = 3;

    /*
     * Enable collision with sensor triggers.
     */
    public static final int SENSOR_TRIGGER = 4;

    /*
     * Enable collision with characters.
     */
    public static final int CHARACTER_GROUP = 5;

    /*
     * Enable collision with everything.
     */
    public static final int COLLISION_ALL_FILTER  = -1;

    public static final int NEXT_COLLISION_GROUP  = 6;


    // By default a rigid body that belongs to a group collides only with other rigid
    // body of the same group.
    private int[] mCollisionFilterMasks =
    {
            1 << DEFAULT_GROUP,
            1 << STATIC_GROUP,
            1 << KINEMATIC_GROUP,
            1 << DEBRIS_GROUP,
            1 << SENSOR_TRIGGER,
            1 << CHARACTER_GROUP,
            1 << NEXT_COLLISION_GROUP,
            1 << 7,
            1 << 8,
            1 << 9,
            1 << 10,
            1 << 11,
            1 << 12,
            1 << 13,
            1 << 14,
            1 << 15
    };

    /**
     * Creates a new Collision Matrix.
     */
    public SXRCollisionMatrix() { }

    /**
     * @param groupId A value between 0 and 15 that is a index in the collision matrix.
     * @return (1 << groupId) a short value multiple of 2 that is the internal filter for {#groupId}.
     */
    public static short getCollisionFilterGroup(int groupId)
    {
        return (short) (1 << groupId);
    }

    /**
     * @param groupId A value between 0 and 15 that is a index in the collision matrix.
     * @return Returns a mask to filter all groups that {#groupId} collides with.
     */
    public short getCollisionFilterMask(int groupId)
    {
        if (groupId < 0 || groupId > 15)
        {
            throw new IllegalArgumentException("Group id must be a value between 0 and 15");
        }
        return (short) mCollisionFilterMasks[groupId];
    }

    /**
     * @param groupId A value between 0 and 15 that is a index in the collision matrix.
     * @param mask Mask to filter all groups that {#groupId} collides with.
     */
    public void setCollisionFilterMask(int groupId, short mask)
    {
        if (groupId < 0 || groupId > 15)
        {
            throw new IllegalArgumentException("Group id must be a value between 0 and 15");
        }
        mCollisionFilterMasks[groupId] = mask;
    }

    /**
     * Enable the collision between two group of collision.
     * @param groupA id of the first group to collide with {#groupB}
     * @param groupB id of the second group to collide with {#groupA}
     */
    public void enableCollision(int groupA, int groupB) {
        setCollision(groupA, groupB, true);
    }

    /**
     * Disable the collision between two group of collision.
     * @param groupA id of the first group to disable the collision with {#groupB}
     * @param groupB id of the second group to disable the collision with {#groupA}
     */
    public void disableCollision(int groupA, int groupB) {
        setCollision(groupA, groupB, false);
    }

    private void setCollision(int groupA, int groupB, boolean enabled)
    {
        if (groupA < 0 || groupA > 15 || groupB < 0 || groupB > 15)
        {
            throw new IllegalArgumentException("Group id must be a value between 0 and 15");
        }

        if (enabled)
        {
            mCollisionFilterMasks[groupA] |= getCollisionFilterGroup(groupB);
            mCollisionFilterMasks[groupB] |= getCollisionFilterGroup(groupA);
        }
        else
        {
            mCollisionFilterMasks[groupA] &= -getCollisionFilterGroup(groupB) - 1;
            mCollisionFilterMasks[groupB] &= -getCollisionFilterGroup(groupA) - 1;
        }
    }
}
