package com.acmerobotics.roadrunner.ftc

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorController
import com.qualcomm.robotcore.hardware.DcMotorSimple

class LocalizationSensorEncoder(
    private val sensor: LocalizationSensor,
    private val usePerpendicular: Boolean,
    private val anyDummyMotor: DcMotor
) : Encoder {

    override var direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD

    override fun getPositionAndVelocity(): PositionVelocityPair {
        // this will run twice when accessing both directions, which isn't ideal for loop times
        // however all tuners only use it once so it's fine
        sensor.updatePoseVel()
        val absPose = sensor.cachedPose
        val absVel = sensor.cachedVel
        /*
        val rotatedPosVector = absPose.heading.inverse().times(absPose.position) // this doesn't work btw, not sure why
        val rotatedVelVector = absPose.heading.inverse().times(absVel.linearVel)
        */
        val rotatedPosVector = absPose.position
        val rotatedVelVector = absVel.linearVel
        val pos: Double
        val vel: Double

        if (usePerpendicular) {
            // y = strafe = perpendicular
            pos = rotatedPosVector.y
            vel = rotatedVelVector.y
        } else {
            pos = rotatedPosVector.x
            vel = rotatedVelVector.x
        }
        return PositionVelocityPair(pos, vel, pos, vel)
    }

    override val controller: DcMotorController // I hate this
        get() = anyDummyMotor.controller
}