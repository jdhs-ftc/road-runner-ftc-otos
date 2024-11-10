package com.acmerobotics.roadrunner.ftc

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D

// SDK Pose
fun Pose2D.toRRPose(): Pose2d {
    return Pose2d(this.xInch, this.yInch, this.headingRad)
}

val Pose2D.xInch: Double
    get() = this.getX(DistanceUnit.INCH)

val Pose2D.yInch: Double
    get() = this.getY(DistanceUnit.INCH)

val Pose2D.headingRad: Double
    get() = this.getHeading(AngleUnit.RADIANS)


// RR Pose
fun Pose2d.toFTCPose(): Pose2D {
    return Pose2D(DistanceUnit.INCH, this.position.x, this.position.y, AngleUnit.RADIANS, this.heading.toDouble())
}

/**
 * Convert Road Runner Pose to OTOS Pose
 * Note: returned OTOS Pose will be in inches and radians
 */
fun Pose2d.toOTOSPose(): SparkFunOTOS.Pose2D {
    return SparkFunOTOS.Pose2D(this.position.x, this.position.y, this.heading.toDouble())
}

// OTOS Pose
/**
 * Convert OTOS Pose to Roadrunner Pose
 * Note: OTOS pose MUST be in inches and radians for this to work
 */
fun SparkFunOTOS.Pose2D.toRRPose(): Pose2d {
    assert(this.h < 2 * Math.PI) { "OTOS pose heading greater than 2 pi (it's ${this.h}); is it in radians?" }
    return Pose2d(this.x, this.y, this.h)
}
