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

package com.samsungxr;

import com.samsungxr.io.SXRCursorController;

/**
 * This class provides all the information corresponding to events generated by
 * a {@link SXRSensor}.
 * <p>
 * A sensor recieves all the pick events from the colliders attached to its
 * descendants and emits corresponding sensor events from the scene object
 * that owns it. Clients can add a listener to the event receiver of that
 * scene object to handle the sensor events.
 * @see ISensorEvents
 * @see SXRNode#getEventReceiver()
 * @see SXREventReceiver#addListener(IEvents)
 */
public class SensorEvent {
    private static final String TAG = SensorEvent.class.getSimpleName();
    private boolean isActive;
    private boolean isOver;
    private SXRPicker.SXRPickedObject pickedObject;
    private SXRCursorController controller;

    // We take a leaf out of the MotionEvent book to implement linked
    // recycling of objects.
    private static final int MAX_RECYCLED = 10;
    private static final Object recyclerLock = new Object();

    private static int recyclerUsed;
    private static SensorEvent recyclerTop;
    private SensorEvent next;

    SensorEvent(){

    }

    /**
     * Set the active flag on the {@link SensorEvent}.
     * This indicates the main button is pressed and the sensor
     * is being "touched".
     * @param isActive
     *            The active flag value.
     */
    void setActive(boolean isActive) {
        this.isActive = isActive;
    }

    /**
     * 
     * Set the {@link SXRCursorController} on the {@link SensorEvent}.
     * Indicates which cursor controller generated the sensor event.
     * 
     * @param controller
     *            The {@link SXRCursorController} that created this event
     */
    void setCursorController(SXRCursorController controller) {
        this.controller = controller;
    }

    /**
     * Set the picking information for the {@link SXRNode} that
     * triggered this {@link SensorEvent} containing the {@link SXRCollider}
     * that was hit.
     * 
     * @param pickedObject
     *            The picking information of the affected {@link SXRNode}.
     */
    void setPickedObject(SXRPicker.SXRPickedObject pickedObject) {
        this.pickedObject = pickedObject;
    }

    /**
     * This flag denotes that the {@link SXRCursorController} "is over" the
     * affected pickedObject.
     * 
     * @param isOver
     *            The value of the "is over" flag.
     */
    void setOver(boolean isOver) {
        this.isOver = isOver;
    }

    /**
     * Retrieves the picking information of the affected {@link SXRNode},
     * including the {@link SXRCollider} that was hit, the hit point and
     * the distance from the origin.
     * 
     * @return The {@link SXRPicker.SXRPickedObject} corresponding to the {@link SXRNode}
     *         that caused this {@link SensorEvent} to be triggered.
     */
    public SXRPicker.SXRPickedObject getPickedObject() {
        return pickedObject;
    }

    /**
     * Use this flag to detect if the input "is over" the {@link SXRNode}
     * 
     * @return <code>true</code> if the input is over the corresponding
     *         {@link SXRNode}. The {@link ISensorEvents} delivers
     *         multiple sensor events when this state is <code>true</code> and
     *         only one event when this state is <code>false</code>.
     * 
     */
    public boolean isOver() {
        return isOver;
    }

    /**
     * Returns the active status of the {@link SensorEvent}.
     * 
     * @return <code>true</code> when the provided input has an active state and
     *         <code>false</code> otherwise.
     * 
     *         This usually denotes a button press on a given input event. The
     *         actual interaction that causes the active state is defined by the
     *         input provided to the {@link SXRInputManager}.
     */
    public boolean isActive() {
        return isActive;
    }

    /**
     * Returns the {@link SXRCursorController} that generated this event.
     * 
     * @return the {@link SXRCursorController} object.
     */
    public SXRCursorController getCursorController() {
        return controller;
    }

    /**
     * Use this method to return a {@link SensorEvent} for use.
     * 
     * @return the {@link SensorEvent} object.
     */
    static SensorEvent obtain() {
        final SensorEvent event;
        synchronized (recyclerLock) {
            event = recyclerTop;
            if (event == null) {
                return new SensorEvent();
            }
            recyclerTop = event.next;
            recyclerUsed -= 1;
        }
        event.next = null;
        return event;
    }

    /**
     * Recycle the {@link SensorEvent} object.
     * 
     * Make sure that the object is not used after this call.
     */
    final void recycle() {
        synchronized (recyclerLock) {
            if (recyclerUsed < MAX_RECYCLED) {
                recyclerUsed++;
                next = recyclerTop;
                recyclerTop = this;
            }
        }
    }
}