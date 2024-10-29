package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorController
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import kotlin.math.cos
import kotlin.math.sin

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

class OtosImu(private val otos: SparkFunOTOS): IMU {
    override fun initialize(parameters: IMU.Parameters?): Boolean {
        // ignore input parameters
        otos.angularUnit = AngleUnit.RADIANS
        return true
    }

    override fun resetYaw() {
        val curPos = otos.position
        otos.position = SparkFunOTOS.Pose2D(curPos.x,curPos.y,0.0)
    }

    override fun getRobotYawPitchRollAngles(): YawPitchRollAngles {
        return YawPitchRollAngles(AngleUnit.RADIANS,otos.position.h,0.0,0.0,System.nanoTime())
    }

    override fun getRobotOrientation(
        reference: AxesReference?,
        order: AxesOrder?,
        angleUnit: AngleUnit?
    ): Orientation? {
        return Orientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS, 0f,otos.position.h.toFloat(), 0f, System.nanoTime())
            .toAxesReference(reference)
            .toAxesOrder(order)
            .toAngleUnit(angleUnit)}

    override fun getRobotOrientationAsQuaternion(): Quaternion {
        return eulerToQuaternion(otos.position.h)
    }

    override fun getRobotAngularVelocity(angleUnit: AngleUnit?): AngularVelocity {
        return AngularVelocity(AngleUnit.RADIANS, 0.0f,otos.velocity.h.toFloat(),0.0f,System.nanoTime()).toAngleUnit(angleUnit)
    }

    override fun getManufacturer(): HardwareDevice.Manufacturer {
        return otos.manufacturer
    }

    override fun getDeviceName(): String {
        return otos.deviceName
    }

    override fun getConnectionInfo(): String {
        return otos.connectionInfo
    }

    override fun getVersion(): Int {
        return otos.version
    }

    override fun resetDeviceConfigurationForOpMode() {
        otos.resetDeviceConfigurationForOpMode()
    }

    override fun close() {
        otos.close()
    }

}
// https://math.stackexchange.com/questions/2975109/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr
fun eulerToQuaternion(yaw: Double): Quaternion {
    val qx =  cos(yaw/2) - sin(yaw/2)
    val qy = cos(yaw/2) + sin(yaw/2)
    val qz = sin(yaw/2) - cos(yaw/2)
    val qw = cos(yaw/2) + sin(yaw/2)
    return Quaternion(qw.toFloat(), qx.toFloat(), qy.toFloat(), qz.toFloat(), System.nanoTime())
}

fun OTOSPoseToRRPose(otosPose: SparkFunOTOS.Pose2D): Pose2d {
    return Pose2d(otosPose.x,otosPose.y,otosPose.h)
}

fun RRPoseToOTOSPose(rrPose: Pose2d): SparkFunOTOS.Pose2D {
    return SparkFunOTOS.Pose2D(rrPose.position.x, rrPose.position.y, rrPose.heading.toDouble())
}