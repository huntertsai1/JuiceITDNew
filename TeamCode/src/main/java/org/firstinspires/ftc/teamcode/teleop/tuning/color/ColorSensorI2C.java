package org.firstinspires.ftc.teamcode.teleop.tuning.color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.hardware.BrushlandColorSensor;

@TeleOp(group = "competition")
@Config
@Disabled
public class ColorSensorI2C extends LinearOpMode {
    RevColorSensorV3 sensor;
    RevColorSensorV3 sensor2;
    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.get(RevColorSensorV3.class, "colorSensorHead");
        sensor2 = hardwareMap.get(RevColorSensorV3.class, "colorSensorTail");
        ((LynxI2cDeviceSynch) sensor.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        ((LynxI2cDeviceSynch) sensor2.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            NormalizedRGBA color = sensor.getNormalizedColors();
            telemetry.addData("rh", color.red);
            telemetry.addData("gh", color.green);
            telemetry.addData("bh", color.blue);
            NormalizedRGBA color2 = sensor2.getNormalizedColors();
            telemetry.addData("rt", color2.red);
            telemetry.addData("gt", color2.green);
            telemetry.addData("bt", color2.blue);
            telemetry.addData("distance head", sensor.getDistance(DistanceUnit.MM));
            telemetry.addData("distance tail", sensor2.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}