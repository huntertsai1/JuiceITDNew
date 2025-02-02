package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.util.hardware.Motor;

@TeleOp(name="cv tuner")
@Config
//Disabled
public class LocalizationTest extends LinearOpMode {
    Motor backLeft;
    Motor backRight;
    Motor frontLeft;
    Motor frontRight;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        otos.setLinearUnit(DistanceUnit.INCH);
//        otos.setAngularUnit(AngleUnit.RADIANS);
//        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(otosXOffset, otosYOffset, otosHeadingOffset);
//        otos.setOffset(offset);
//        otos.setLinearScalar(1.0);
//        otos.setAngularScalar(1.0);
//        otos.calibrateImu();
//        otos.resetTracking();
//        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(startingX, startingY, startingHeading);
//        otos.setPosition(currentPosition);
//        // Get the hardware and firmware version
//        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
//        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
//        otos.getVersionInfo(hwVersion, fwVersion);

        backLeft = new Motor(3, "leftBack", hardwareMap, true);
        backRight = new Motor(3, "rightBack", hardwareMap, false);
        frontLeft = new Motor(3, "leftFront", hardwareMap, true);
        frontRight = new Motor(3, "rightFront", hardwareMap, false);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {

            double x,y,rx;
            x = -gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            rx = gamepad1.right_stick_x;
            setDrivePower(-x, y, rx);

        }
    }

    public void setDrivePower(double x, double y, double rx) {
        double powerFrontLeft = y + x + rx;
        double powerFrontRight = y - x - rx;
        double powerBackLeft = (y - x + rx) * -1;
        double powerBackRight = (y + x - rx) * -1;

        if (Math.abs(powerFrontLeft) > 1 || Math.abs(powerBackLeft) > 1 ||
                Math.abs(powerFrontRight) > 1 || Math.abs(powerBackRight) > 1) {
            // Find the largest power
            double max;
            max = Math.max(Math.abs(powerFrontLeft), Math.abs(powerBackLeft));
            max = Math.max(Math.abs(powerFrontRight), max);
            max = Math.max(Math.abs(powerBackRight), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            powerFrontLeft /= max;
            powerBackLeft /= max;
            powerFrontRight /= max;
            powerBackRight /= max;
        }

        frontLeft.setSpeed((float)powerFrontLeft);
        frontRight.setSpeed((float)powerFrontRight);
        backLeft.setSpeed(-(float)powerBackLeft);
        backRight.setSpeed(-(float)powerBackRight);
    }
}
