package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;
import org.firstinspires.ftc.teamcode.util.hardware.BrushlandColorSensor;
import org.firstinspires.ftc.teamcode.util.hardware.ContinuousServo;

@TeleOp(group = "competition")
@Config
@Disabled
public class REVColorSensorTest extends LinearOpMode {
    RevColorSensorV3 sensor;
    ContinuousServo servo1;
    ContinuousServo servo2;
    public static boolean CLAW_ENABLED = false;
    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        sensor.initialize();
        servo1 = new ContinuousServo(1, "claw1", hardwareMap);
        servo2 = new ContinuousServo(1, "claw2", hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            if (CLAW_ENABLED) {
                servo1.servo.setPower(1);
                servo2.servo.setPower(-1);
            } else {
                servo1.servo.setPower(0);
                servo2.servo.setPower(0);
            }

            NormalizedRGBA color = sensor.getNormalizedColors();

            float red = color.red;
            float green = color.green;
            float blue = color.blue;
            SampleColors detected = null;
            if (blue > 0.012) {
                // Extreme blue output -> blue sample
                detected = SampleColors.BLUE;
            } else if (red >= 0.01 && green <= 0.01) {
                detected = SampleColors.RED;
            } else if (red >= 0.02 && green > 0.02) {
                detected = SampleColors.YELLOW;
            }


            telemetry.addData("detected", detected);
            telemetry.addData("r", red);
            telemetry.addData("g", green);
            telemetry.addData("b", blue);
            telemetry.addData("distance", sensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}