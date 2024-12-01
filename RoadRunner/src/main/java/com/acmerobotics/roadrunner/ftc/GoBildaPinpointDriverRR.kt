/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */
package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver.GoBildaOdometryPods
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles

@I2cDeviceType
@DeviceProperties(
    name = "goBILDA® Pinpoint Odometry Computer Roadrunner Driver",
    xmlTag = "goBILDAPinpointRR",
    description = "goBILDA® Pinpoint Odometry Computer (IMU Sensor Fusion for 2 Wheel Odometry)"
)
class GoBildaPinpointDriverRR(deviceClient: I2cDeviceSynchSimple, deviceClientIsOwned: Boolean) :
    GoBildaPinpointDriver(deviceClient, deviceClientIsOwned), IMU {
    var currentTicksPerMM = 0f

    companion object {
        const val goBILDA_SWINGARM_POD = 13.26291192f //ticks-per-mm for the goBILDA Swingarm Pod
        const val goBILDA_4_BAR_POD = 19.89436789f //ticks-per-mm for the goBILDA 4-Bar Pod
    }

    /**
     * If you're using goBILDA odometry pods, the ticks-per-mm values are stored here for easy access.
     *
     * @param pods goBILDA_SWINGARM_POD or goBILDA_4_BAR_POD
     */
    override fun setEncoderResolution(pods: GoBildaOdometryPods) {
        super.setEncoderResolution(pods)
        currentTicksPerMM =
            if (pods == GoBildaOdometryPods.goBILDA_SWINGARM_POD) {
                goBILDA_SWINGARM_POD
            } else if (pods == GoBildaOdometryPods.goBILDA_4_BAR_POD) {
                goBILDA_4_BAR_POD
            } else {
                throw NotImplementedError("This odometry type not implemented in Roadrunner Pinpoint Integration")
            }
    }

    /**
     * Sets the encoder resolution in ticks per mm of the odometry pods. <br></br>
     * You can find this number by dividing the counts-per-revolution of your encoder by the circumference of the wheel.
     *
     * @param ticks_per_mm should be somewhere between 10 ticks/mm and 100 ticks/mm a goBILDA Swingarm pod is ~13.26291192
     */
    override fun setEncoderResolution(ticks_per_mm: Double) {
        super.setEncoderResolution(ticks_per_mm)
        currentTicksPerMM = ticks_per_mm.toFloat()
    }


    /**
     * Added by j5155 (comments from Gobilda)
     *
     *
     * Send a position that the Pinpoint should use to track your robot relative to. You can use this to
     * update the estimated position of your robot with new external sensor data, or to run a robot
     * in field coordinates.
     *
     * This overrides the current position.
     *
     * **Using this feature to track your robot's position in field coordinates:**
     *
     * When you start your code, send a Pose2D that describes the starting position on the field of your robot.
     * Say you're on the red alliance, your robot is against the wall and closer to the audience side,
     * and the front of your robot is pointing towards the center of the field.
     * You can send a setPosition with something like -600mm x, -1200mm Y, and 90 degrees. The pinpoint would then always
     * keep track of how far away from the center of the field you are.
     *
     *
     * **Using this feature to update your position with additional sensors:**
     *
     * Some robots have a secondary way to locate their robot on the field. This is commonly
     * Apriltag localization in FTC, but it can also be something like a distance sensor.
     * Often these external sensors are absolute (meaning they measure something about the field)
     * so their data is very accurate. But they can be slower to read, or you may need to be in a very specific
     * position on the field to use them. In that case, spend most of your time relying on the Pinpoint
     * to determine your location. Then when you pull a new position from your secondary sensor,
     * send a setPosition command with the new position. The Pinpoint will then track your movement
     * relative to that new, more accurate position.
     *
     * @param pos a Pose2D describing the robot's new position.
     */
    fun setPosition(pos: Pose2d): Pose2d {
        setPosition(
            Pose2D(
                DistanceUnit.INCH,
                pos.position.x,
                pos.position.y,
                AngleUnit.RADIANS,
                pos.heading.toDouble()
            )
        )
        return pos
    }

    /**
     * Added by j5155
     * A Roadrunner Pose2D containing the estimated position of the robot
     * Getting reads the cached position since the last update
     * Setting writes the new position to the Pinpoint immediately
     */
    var positionRR: Pose2d
        get() = Pose2d(
            position.getX(DistanceUnit.INCH),
            position.getY(DistanceUnit.INCH),
            position.getHeading(AngleUnit.RADIANS)
        )
        set(newPose) {
            position = Pose2D(
                DistanceUnit.INCH,
                newPose.position.x,
                newPose.position.y,
                AngleUnit.RADIANS,
                newPose.heading.toDouble()
            )
        }


    /**
     * Added by j5155
     *
     * @return a Roadrunner PoseVelocity2D containing the estimated velocity of the robot, velocity is unit per second
     */
    val velocityRR: PoseVelocity2d
        get() = PoseVelocity2d(
            Vector2d(
                velocity.getX(DistanceUnit.INCH),
                velocity.getY(DistanceUnit.INCH)
            ),
            velocity.getHeading(AngleUnit.RADIANS)
        )


    // IMU implementation (untested, does not update in itself (that might cause problems))

    /**
     * Does nothing
     *
     */
    override fun initialize(parameters: IMU.Parameters): Boolean {
        return true
    }

    override fun resetYaw() {
        val curPos = positionRR
        positionRR = Pose2d(curPos.position, Rotation2d.fromDouble(0.0))
    }

    override fun getRobotYawPitchRollAngles(): YawPitchRollAngles {
        return YawPitchRollAngles(
            AngleUnit.RADIANS,
            position.getHeading(AngleUnit.RADIANS),
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
            position.getHeading(AngleUnit.RADIANS).toFloat(),
            System.nanoTime()
        )
            .toAxesReference(reference)
            .toAxesOrder(order)
            .toAngleUnit(angleUnit)
    }

    override fun getRobotOrientationAsQuaternion(): Quaternion {
        return eulerToQuaternion(position.getHeading(AngleUnit.RADIANS))
    }

    override fun getRobotAngularVelocity(angleUnit: AngleUnit): AngularVelocity {
        return AngularVelocity(
            AngleUnit.RADIANS,
            0.0f,
            0.0f,
            headingVelocity.toFloat(),
            System.nanoTime()
        ).toAngleUnit(angleUnit)
    }
}






