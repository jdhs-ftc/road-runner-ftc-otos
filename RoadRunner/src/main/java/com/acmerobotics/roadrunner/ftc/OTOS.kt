package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ftc.SparkFunOTOS.Pose2D
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorController
import com.qualcomm.robotcore.hardware.DcMotorSimple

class OtosEncoder(private val otos: SparkFunOTOS, private val useYDirection: Boolean, private val anyDummyMotor: DcMotor) : Encoder {
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
        return PositionVelocityPair(
            pos,
            vel,
            pos,
            vel
        )
    }

    override val controller: DcMotorController
        get() = anyDummyMotor.controller
}

fun OTOSPoseToRRPose(otosPose: Pose2D): Pose2d {
    return Pose2d(otosPose.x,otosPose.y,otosPose.h)
}