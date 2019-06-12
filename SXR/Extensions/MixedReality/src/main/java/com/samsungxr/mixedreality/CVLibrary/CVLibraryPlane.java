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

//import com.google.ar.core.Plane;
import com.google.ar.core.Pose;
import android.support.annotation.NonNull;

import com.samsungxr.SXRCameraRig;
import com.samsungxr.SXRContext;
import com.samsungxr.SXRNode;
import com.samsungxr.SXRTransform;
import com.samsungxr.mixedreality.SXRAnchor;
import com.samsungxr.mixedreality.SXRHitResult;
import com.samsungxr.mixedreality.SXRPlane;
import com.samsungxr.mixedreality.SXRTrackingState;

import org.joml.Intersectionf;
import org.joml.Vector3f;


import java.nio.FloatBuffer;


class CVLibraryPlane extends SXRPlane {
    private final float[] mPose = new float[16];
    private final CVLibrarySession mSession;
    private final Vector3f mPlaneNormal;

    protected CVLibraryPlane(SXRContext gvrContext, CVLibrarySession session) {
        super(gvrContext);
        mSession = session;
        mPose[0] = 1;
        mPose[5] = 1;
        mPose[10] = 1;
        mPose[15] = 1;
        mPlaneNormal = new Vector3f(0, 1, 0);
        mPlaneType = Type.HORIZONTAL_UPWARD_FACING;
    }

    public void getCenterPose(@NonNull float[] poseOut)
    {
        System.arraycopy(mPose, 0, poseOut, 0, 16);
    }

    @Override
    public SXRTrackingState getTrackingState() {
        return mTrackingState;
    }

    @Override
    public Type getPlaneType() {
        return mPlaneType;
    }

    @Override
    public float getWidth()
    {
        return mSession.getARToVRScale();
    }


    @Override
    public float getHeight()
    {
        return mSession.getARToVRScale();
    }

    @Override
    public float[] get3dPolygonAsArray()
    {
        float s = mSession.getARToVRScale();
        return new float[] { -s, 0, -s, s, 0, -s, s, 0, s, -s, 0, s };
    }

    @Override
    public SXRPlane getParentPlane() {
        return mParentPlane;
    }

    SXRHitResult processHit(Vector3f rayStart, Vector3f rayDir)
    {
        float t = Intersectionf.intersectRayPlane(
                rayStart.x, rayStart.y, rayStart.z,
                rayDir.x, rayDir.y, rayDir.z,
                0, 0, 0,
                mPlaneNormal.x, mPlaneNormal.y, mPlaneNormal.z, 0.0001f);
        if (t > 0)
        {
            SXRHitResult hit = new SXRHitResult();
            float hitx = t * rayDir.x;
            float hity = t * rayDir.y;
            float hitz = t * rayDir.z;
            float s = mSession.getARToVRScale();
            float[] pose = new float[16];

            double d = Math.sqrt(hitx * hitx + hity * hity + hitz * hitz);
            hitx += rayStart.x;
            hity += rayStart.y;
            hitz += rayStart.z;
            getCenterPose(pose);
            pose[12] = s * hitx;
            pose[13] = s * hity;
            pose[14] = s * hitz;
            hit.setPlane(this);
            hit.setDistance((float) d * s);
            return hit;
        }
        return null;
    }

    @Override
    public SXRAnchor createAnchor(float[] pose, SXRNode owner)
    {
        return mSession.createAnchor(pose, owner);
    }

    public boolean isPoseInPolygon(float[] pose)
    {
        return false;
    }

}
