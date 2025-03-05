package com.acmerobotics.roadrunner.ftc.octoquad

import android.util.Log
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.LocalizationSensorIMU
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.roundToInt

@I2cDeviceType
@DeviceProperties(xmlTag = "OctoQuadFTC_FW_v3_RR", name = "OctoQuadFTC MK2 Roadrunner Driver")
class OctoQuadRR(deviceClient: I2cDeviceSynchSimple, deviceClientIsOwned: Boolean):
    OctoQuadFWv3(deviceClient, deviceClientIsOwned), LocalizationSensorIMU {
    /** Cached pose since last read */
    override var cachedPose: Pose2d = Pose2d(0.0,0.0,0.0)

    /** Cached velocity since last read */
    override var cachedVel: PoseVelocity2d = PoseVelocity2d(Vector2d(0.0,0.0),0.0)

    /** Read the sensor to update pose and vel (will be run every loop) */
    override fun updatePoseVel() {
        val localizer = OctoQuadFWv3.LocalizerDataBlock()
        readLocalizerData(localizer)
        if (cachedVel.linearVel.norm() > 100 || localizer.localizerStatus != OctoQuadFWv3.LocalizerStatus.RUNNING
            || !localizer.crcOk
        ) {
            if (cachedVel.linearVel.norm() > 100) {
                println("12087 VELOCITY OUTLIER EVENT")
            } else if (!localizer.crcOk) {
                println("12087 CRC NOT OK")
            } else {
                println("12087 LOCALIZER STATUS OUTLIER EVENT")
            }
            println("12087 velocity: (in/s) ${cachedVel.linearVel.x} x, ${cachedVel.linearVel.y} y, (rad/s) ${cachedVel.angVel} heading")
            println("12087 position: (in) ${cachedPose.position.x} x, ${cachedPose.position.y} y, (rad) ${cachedPose.heading}")
            println("12087 localizer status: ${localizer.localizerStatus}")
            return // throw away bad data
        }
        cachedPose = Pose2d(
            DistanceUnit.INCH.fromMm(localizer.posX_mm.toDouble()),
            DistanceUnit.INCH.fromMm(localizer.posY_mm.toDouble()),
            localizer.heading_rad.toDouble())
        cachedVel = PoseVelocity2d(
            Vector2d(
                DistanceUnit.INCH.fromMm(localizer.velX_mmS.toDouble()),
                DistanceUnit.INCH.fromMm(localizer.velY_mmS.toDouble())),
            localizer.velHeading_radS.toDouble())



    }

    /** Write an updated pose to the sensor and to the cached value */
    override fun writePose(pose: Pose2d) {
        setLocalizerPose(
            DistanceUnit.MM.fromInches(pose.position.x).roundToInt(),
            DistanceUnit.MM.fromInches(pose.position.y).roundToInt(),
            pose.heading.toDouble().toFloat()
        )
    }

    /** Initialize common/base aspects of the sensor, like setting the units and calibrating the IMU.
     *
     * Does not include any user/sensor specific tuning constants.
     *
     * Block until IMU calibration ends. */
    override fun baseInitialize() {
        resetLocalizerAndCalibrateIMU()
        val timeout = ElapsedTime()
        var currentStatus = localizerStatus // localizerStatus reads from hardware
        while (currentStatus != OctoQuadFWv3.LocalizerStatus.RUNNING) {
            Thread.sleep(10)
            if (timeout.seconds() > 3) {
                Log.println(
                    Log.WARN,
                    "OctoQuadRR",
                    "init: Calibration timeout reached, OQ still not ready. OctoQuad reports $currentStatus"
                )
                RobotLog.addGlobalWarningMessage(
                    "OctoQuad still not ready, timeout reached. OctoQuad reports $currentStatus." +
                            "Continuing anyway, good luck!"
                )
                break
            }
            currentStatus = localizerStatus
        }
    }

}