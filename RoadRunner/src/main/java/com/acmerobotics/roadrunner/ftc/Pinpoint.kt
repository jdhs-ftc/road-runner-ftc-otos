package com.acmerobotics.roadrunner.ftc

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorController
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class PinpointEncoder(
    private val pinpoint: GoBildaPinpointDriver,
    private val usePerpendicular: Boolean,
    private val reversed: Boolean,
    private val anyDummyMotor: DcMotor
) : Encoder {
    override var direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD

    override fun getPositionAndVelocity(): PositionVelocityPair {
        pinpoint.bulkUpdate()
        val pos: Double
        val vel: Double
        // kotlin wraps getPosition here
        // theoretically this could be worse if getPosition did a read, but it only checks the cached value from last bulk read
        // so this is (currently) fine
        if (usePerpendicular) {
            // y = strafe = perpendicular
            // have to convert to inch units manually from millimeters
            // also have to calculate the distance in millimeters manually
            pos = DistanceUnit.INCH.fromMm((pinpoint.encoderY / pinpoint.currentTicksPerMM).toDouble())
        } else {
            pos = DistanceUnit.INCH.fromMm((pinpoint.encoderX / pinpoint.currentTicksPerMM).toDouble())
        }

        // yes, velocity.position is weird.
        // it's returning a pose2d in inches/sec as vel rn
        return PositionVelocityPair(pos, vel, pos, vel)
    }

    override val controller: DcMotorController // i hate this
        get() = anyDummyMotor.controller
}