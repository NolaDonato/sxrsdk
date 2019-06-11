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

package com.samsungxr.mixedreality.CVLibrary;

//import com.google.ar.core.AugmentedImage;
import com.samsungxr.SXRCameraRig;
import com.samsungxr.SXRContext;
import com.samsungxr.SXRNode;
import com.samsungxr.SXRTransform;
import com.samsungxr.mixedreality.SXRAnchor;
import com.samsungxr.mixedreality.SXRMarker;
import com.samsungxr.mixedreality.SXRTrackingState;

import org.joml.Vector3f;

/**
 * Represents an ARCore Augmented Image
 */
public class CVLibraryMarker extends SXRMarker
{
    private final CVLibrarySession mSession;
    private final String mName;
    private final float[] mPose = new float[16];

    //private AugmentedImage mAugmentedImage;

    protected CVLibraryMarker(CVLibrarySession session, String name) {
        super(session.getContext());
        mSession = session;
        mName = name;
        mPose[0] = 1;
        mPose[5] = 1;
        mPose[10] = 1;
        mPose[15] = 1;
        mTrackingState = SXRTrackingState.PAUSED;
    }

    public String getName() { return mName; }

    /**
     * @return Returns the estimated width
     */
    @Override
    public float getExtentX()
    {
        return 0.1f * mSession.getARToVRScale();
    }

    /**
     * @return Returns the estimated height
     */
    @Override
    public float getExtentZ()
    {
        return 0.1f * mSession.getARToVRScale();
    }

    /**
     * @return The augmented image center pose
     */
    @Override
    public final float[] getCenterPose()
    {
        return mPose;
    }

    @Override
    public SXRAnchor createAnchor(SXRNode owner)
    {
        return mSession.createAnchor(getCenterPose(), owner);
    }

    /**
     *
     * @return The tracking state
     */
    @Override
    public SXRTrackingState getTrackingState() {
        return mTrackingState;
    }

    public void update(SXRCameraRig rig)
    {
        SXRNode owner = getOwnerObject();
        if (owner != null)
        {
            SXRTransform trig = rig.getHeadTransform();
            SXRTransform tnode = owner.getTransform();
            mPose[12] = -trig.getPositionX();
            mPose[13] = -trig.getPositionY();
            mPose[14] = -trig.getPositionZ();
            tnode.setModelMatrix(mPose);
        }
    }

    /**
     * Set the augmented image tracking state
     *
     * @param state
     */
    protected void setTrackingState(SXRTrackingState state) {
        mTrackingState = state;
    }
}
