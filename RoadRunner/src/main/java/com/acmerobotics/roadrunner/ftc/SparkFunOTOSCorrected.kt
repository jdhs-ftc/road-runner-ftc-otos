package com.acmerobotics.roadrunner.ftc

import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.I2cDeviceSynch
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import java.util.Arrays
import kotlin.math.cos
import kotlin.math.sin

@I2cDeviceType
@DeviceProperties(
    name = "SparkFun OTOS Corrected",
    xmlTag = "SparkFunOTOS2",
    description = "SparkFun Qwiic Optical Tracking Odometry Sensor Corrected"
)
class SparkFunOTOSCorrected(deviceClient: I2cDeviceSynch) : SparkFunOTOS(deviceClient), IMU {
    /**
     * Gets only the position and velocity measured by the
     * OTOS in a single burst read
     * @param pos Position measured by the OTOS
     * @param vel Velocity measured by the OTOS
     */
    @Suppress("unused")
    fun getPosVel(pos: Pose2D, vel: Pose2D) {
        // Read all pose registers
        val rawData = deviceClient.read(REG_POS_XL.toInt(), 12)

        // Convert raw data to pose units
        pos.set(regsToPose(Arrays.copyOfRange(rawData, 0, 6), INT16_TO_METER, INT16_TO_RAD))
        vel.set(regsToPose(Arrays.copyOfRange(rawData, 6, 12), INT16_TO_MPS, INT16_TO_RPS))
    }

    // Modified version of poseToRegs to fix the pose setting issue
    // see https://discord.com/channels/225450307654647808/1246977443030368349/1271702497659977760
    override fun poseToRegs(rawData: ByteArray, pose: Pose2D, xyToRaw: Double, hToRaw: Double) {
        // Convert pose units to raw data
        val rawX = (_distanceUnit.toMeters(pose.x) * xyToRaw).toInt().toShort()
        val rawY = (_distanceUnit.toMeters(pose.y) * xyToRaw).toInt().toShort()
        val rawH = (_angularUnit.toRadians(pose.h) * hToRaw).toInt().toShort()

        // Store raw data in buffer
        rawData[0] = (rawX.toInt() and 0xFF).toByte()
        rawData[1] = ((rawX.toInt() shr 8) and 0xFF).toByte()
        rawData[2] = (rawY.toInt() and 0xFF).toByte()
        rawData[3] = ((rawY.toInt() shr 8) and 0xFF).toByte()
        rawData[4] = (rawH.toInt() and 0xFF).toByte()
        rawData[5] = ((rawH.toInt() shr 8) and 0xFF).toByte()
    }

    override fun initialize(parameters: IMU.Parameters): Boolean {
        return true
    }

    // IMU implementation

    override fun resetYaw() {
        val curPos = position
        position = Pose2D(curPos.x,curPos.y,0.0)
    }

    override fun getRobotYawPitchRollAngles(): YawPitchRollAngles {
        return YawPitchRollAngles(angularUnit,position.h,0.0,0.0,System.nanoTime())
    }

    override fun getRobotOrientation(
        reference: AxesReference,
        order: AxesOrder,
        angleUnit: AngleUnit
    ): Orientation {
        return Orientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, angularUnit, 0f,0f,position.h.toFloat(), System.nanoTime())
            .toAxesReference(reference)
            .toAxesOrder(order)
            .toAngleUnit(angleUnit)}

    override fun getRobotOrientationAsQuaternion(): Quaternion {
        return eulerToQuaternion(position.h)
    }

    override fun getRobotAngularVelocity(angleUnit: AngleUnit): AngularVelocity {
        return AngularVelocity(angularUnit, 0.0f,0.0f,velocity.h.toFloat(),System.nanoTime()).toAngleUnit(angleUnit)
    }
}

// https://math.stackexchange.com/questions/2975109/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr
// UNTESTED
fun eulerToQuaternion(yaw: Double): Quaternion {
    val qx = cos(yaw / 2) - sin(yaw / 2)
    val qy = cos(yaw / 2) + sin(yaw / 2)
    val qz = sin(yaw / 2) - cos(yaw / 2)
    val qw = cos(yaw / 2) + sin(yaw / 2)
    return Quaternion(qw.toFloat(), qx.toFloat(), qy.toFloat(), qz.toFloat(), System.nanoTime())
}