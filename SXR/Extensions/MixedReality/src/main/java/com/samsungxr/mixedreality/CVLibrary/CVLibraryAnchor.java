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

//import com.google.ar.core.Anchor;
import com.samsungxr.SXRContext;
import com.samsungxr.SXRNode;
import com.samsungxr.mixedreality.SXRAnchor;
import com.samsungxr.mixedreality.SXRTrackingState;

/**
 * Represents a ARCore anchor in the scene.
 *
 */
public class CVLibraryAnchor extends SXRAnchor
{
    private final float[] mPose = new float[16];

    protected CVLibraryAnchor(SXRContext ctx)
    {
        super(ctx);
        mPose[0] = 1;
        mPose[5] = 1;
        mPose[10] = 1;
        mPose[15] = 1;
    }

    /**
     * Set the anchor tracking state
     *
     * @param state
     */
    protected void setTrackingState(SXRTrackingState state) { mTrackingState = state; }


    @Override
    public SXRTrackingState getTrackingState()
    {
        return mTrackingState;
    }

    @Override
    public String getCloudAnchorId()
    {
        throw new UnsupportedOperationException("Cloud anchors are not supported at this time");
    }

    void update(float[] pose)
    {
        SXRNode owner = getOwnerObject();

        if (owner != null && isEnabled())
        {
            System.arraycopy(pose, 0, mPose, 0, 16);
        }
    }

    public final float[] getPose()
    {
        return mPose;
    }

}
