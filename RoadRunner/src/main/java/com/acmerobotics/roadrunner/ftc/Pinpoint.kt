package com.acmerobotics.roadrunner.ftc

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorController
import com.qualcomm.robotcore.hardware.DcMotorSimple

class PinpointEncoder(
    private val pinpoint: GoBildaPinpointDriver,
    private val useYDirection: Boolean,
    private val reversed: Boolean,
    private val anyDummyMotor: DcMotor
) : Encoder {
    override var direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD

    override fun getPositionAndVelocity(): PositionVelocityPair {
        pinpoint.bulkUpdate()
        // kotlin wraps getPosition here
        // theoretically this could be worse if getPosition did a read, but it only checks the cached value from last bulk read
        // so this is (currently) fine
        val pos: Double = pinpoint.position.position.x
        val vel: Double = pinpoint.velocity.position.x // yes, velocity.position is weird.
        // it's returning a pose2d in inches/sec as vel rn
        return PositionVelocityPair(pos, vel, pos, vel)
    }

    override val controller: DcMotorController // i hate this
        get() = anyDummyMotor.controller
}