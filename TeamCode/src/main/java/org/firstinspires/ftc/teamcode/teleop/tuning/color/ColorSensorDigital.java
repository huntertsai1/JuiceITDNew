package org.firstinspires.ftc.teamcode.teleop.tuning.color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.hardware.BrushlandColorSensor;

@TeleOp(group = "competition")
@Config
//@Disabled
public class ColorSensorDigital extends LinearOpMode {
    BrushlandColorSensor sensor;
    BrushlandColorSensor sensor2;
    @Override
    public void runOpMode() throws InterruptedException {
        sensor = new BrushlandColorSensor(0, "colorSensorHead", hardwareMap);
        sensor2 = new BrushlandColorSensor(0, "colorSensorTail", hardwareMap);
        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("hp0", sensor.getPin0Digital());
            telemetry.addData("hp1", sensor.getPin1());
            telemetry.addData("tp0", sensor2.getPin0Digital());
            telemetry.addData("tp1", sensor2.getPin1());
            telemetry.update();
        }
    }
}