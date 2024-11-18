package com.acmerobotics.roadrunner.ftc

import android.util.Log
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorController
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class PinpointEncoder(
    private val pinpoint: GoBildaPinpointDriverRR,
    private val usePerpendicular: Boolean,
    private val anyDummyMotor: DcMotor
) : Encoder {
    init {
        Log.println(Log.INFO, "PinpointEncoder", "init: Initializing pinpoint encoder in tuning mode")
        Log.println(Log.INFO, "PinpointEncoder", "init: Old yaw scalar = " + pinpoint.yawScalar)
        Log.println(Log.WARN, "PinpointEncoder", "init: Setting Pinpoint yaw scalar to 0. Perform power cycle to reset")
        RobotLog.addGlobalWarningMessage(
            "Disabling Pinpoint IMU. Perform a power cycle (turn the robot off and back on again) to reset it before running Feedback Tuner, LocalizationTest, or an auto (Angular Scalar now 0, previously %f)",
            pinpoint.yawScalar
        )
        // Makes the output from the pinpoint robot centric
        // Officially recommended by Gobilda:
        // https://discord.com/channels/225450307654647808/225451520911605765/1286457799798296669
        pinpoint.setYawScalar(0.0)
        pinpoint.resetPosAndIMU()
    }

    override var direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD

    override fun getPositionAndVelocity(): PositionVelocityPair {
        // this will run twice when accessing both directions, which isn't ideal for loop times
        // however all tuners only use it once so it's fine
        pinpoint.update()
        val pos: Double
        val vel: Double

        if (usePerpendicular) {
            // y = strafe = perpendicular
            pos = pinpoint.positionRR.position.y
            vel = pinpoint.velocityRR.linearVel.y
        } else {
            pos = pinpoint.positionRR.position.x
            vel = pinpoint.velocityRR.linearVel.x
        }
        return PositionVelocityPair(pos, vel, pos, vel)
    }

    override val controller: DcMotorController // I hate this
        get() = anyDummyMotor.controller
}

/**
 * Passes through the raw encoder values of the Pinpoint (after converting to inches)
 * Also calculates speed manually
 * This allows you to access raw encoder values while also running the pinpoint in field centric mode
 * Basically only for cross comparing 2 wheel and pinpoint, NOT RECOMMENDED for any other use
 */
class PinpointRawPassthroughEncoder(
    private val pinpoint: GoBildaPinpointDriverRR,
    private val usePerpendicular: Boolean,
    private val reversed: Boolean,
    private val anyDummyMotor: DcMotor,
    private val autoUpdate: Boolean,
) : Encoder {
    override var direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD

    private var lastPos = 0.0;
    private var lastTime = System.nanoTime();

    // TODO: consider using robot centric pose& velocity instead??
    // set yaw scalar to zero?
    // https://discord.com/channels/225450307654647808/225451520911605765/1286457799798296669
    override fun getPositionAndVelocity(): PositionVelocityPair {
        if (autoUpdate) {
            pinpoint.update()
        }
        var pos: Double // this is a var so the reversing logic works, though it seems kind of weird to me
        val vel: Double

        if (usePerpendicular) {
            // y = strafe = perpendicular
            // have to convert to inch units manually from millimeters
            // also have to calculate the distance in millimeters manually
            pos = DistanceUnit.INCH.fromMm((pinpoint.encoderY / pinpoint.currentTicksPerMM).toDouble())
        } else {
            pos = DistanceUnit.INCH.fromMm((pinpoint.encoderX / pinpoint.currentTicksPerMM).toDouble())
        }

        if (reversed) {
            pos *= -1
        }

        if (lastPos == 0.0) {
            lastPos = pos
            vel = 0.0
        } else {
            // calculate velocity manually
            // the pinpoint does not output velocity at all, unfortunately
            // see https://discord.com/channels/225450307654647808/225451520911605765/1286454383978090549 and
            // https://discord.com/channels/225450307654647808/1286099179617124386/1286316901479350272

            // ideally we would filter this,
            // but this should be fine for now, esp since this should barely be used anyway
            // why I didn't filter it:
            // https://discord.com/channels/225450307654647808/225451520911605765/1286453115868020798
            // and https://discord.com/channels/225450307654647808/225451520911605765/1286453284667789362
            val currentTime = System.nanoTime()
            val timeDiffSec = (currentTime - lastTime) * 1e-9;
            vel = (pos - lastPos) / timeDiffSec
            lastPos = pos
            lastTime = currentTime
        }
        return PositionVelocityPair(pos, vel, pos, vel)
    }

    override val controller: DcMotorController // I hate this
        get() = anyDummyMotor.controller
}