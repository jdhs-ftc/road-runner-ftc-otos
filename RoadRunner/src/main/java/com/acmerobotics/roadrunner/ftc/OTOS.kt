package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorController
import com.qualcomm.robotcore.hardware.DcMotorSimple

class OtosEncoder(private val otos: SparkFunOTOS, private val useYDirection: Boolean, private val reversed: Boolean, private val anyDummyMotor: DcMotor) : Encoder {
    override var direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD

    override fun getPositionAndVelocity(): PositionVelocityPair {
        val pos: Double
        val vel: Double
        if (useYDirection) {
            pos = otos.position.y
            vel = otos.velocity.y
        } else {
            pos = otos.position.x
            vel = otos.velocity.x
        }
        if (reversed) {
            return PositionVelocityPair(
                -pos,
                -vel,
                -pos,
                -vel
            )
        } else {
            return PositionVelocityPair(
                pos,
                vel,
                pos,
                vel
            )
        }
    }

    override val controller: DcMotorController
        get() = anyDummyMotor.controller
}

fun OTOSPoseToRRPose(otosPose: SparkFunOTOS.Pose2D): Pose2d {
    return Pose2d(otosPose.x,otosPose.y,otosPose.h)
}

fun RRPoseToOTOSPose(rrPose: Pose2d): SparkFunOTOS.Pose2D {
    return SparkFunOTOS.Pose2D(rrPose.position.x, rrPose.position.y, rrPose.heading.toDouble())
}