package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d

interface LocalizationSensor {
    /** Cached pose since last read */
    var cachedPose: Pose2d

    /** Cached velocity since last read */
    var cachedVel: PoseVelocity2d

    /** Read the sensor to update pose and vel (will be run every loop) */
    fun updatePoseVel()

    /** Write an updated pose to the sensor and to the cached value */
    fun writePose(pose: Pose2d)

    /** Initialize common/base aspects of the sensor, like setting the units and calibrating the IMU.
     *
     * Does not include any user/sensor specific tuning constants.
     *
     * Block until IMU calibration ends. */
    fun baseInitialize()
}