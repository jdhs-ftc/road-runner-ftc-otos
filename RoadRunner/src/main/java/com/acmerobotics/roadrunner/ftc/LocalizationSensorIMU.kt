package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Rotation2d
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles

interface LocalizationSensorIMU: LocalizationSensor, IMU {
    // IMU implementation (does not update in itself, that might cause problems)

    /**
     * Does nothing
     *
     */
    override fun initialize(parameters: IMU.Parameters): Boolean {
        return true
    }

    override fun resetYaw() {
        val curPos = cachedPose
        writePose(Pose2d(curPos.position, Rotation2d.fromDouble(0.0)))
    }

    override fun getRobotYawPitchRollAngles(): YawPitchRollAngles {
        return YawPitchRollAngles(
            AngleUnit.RADIANS,
            cachedPose.heading.toDouble(),
            0.0,
            0.0,
            System.nanoTime()
        )
    }

    override fun getRobotOrientation(
        reference: AxesReference,
        order: AxesOrder,
        angleUnit: AngleUnit
    ): Orientation {
        return Orientation(
            AxesReference.EXTRINSIC,
            AxesOrder.XYZ,
            AngleUnit.RADIANS,
            0f,
            0f,
            cachedPose.heading.toDouble().toFloat(),
            System.nanoTime()
        )
            .toAxesReference(reference)
            .toAxesOrder(order)
            .toAngleUnit(angleUnit)
    }

    override fun getRobotOrientationAsQuaternion(): Quaternion {
        return eulerToQuaternion(cachedPose.heading.toDouble())
    }

    override fun getRobotAngularVelocity(angleUnit: AngleUnit): AngularVelocity {
        return AngularVelocity(
            AngleUnit.RADIANS,
            0.0f,
            0.0f,
            cachedVel.angVel.toFloat(),
            System.nanoTime()
        ).toAngleUnit(angleUnit)
    }
}