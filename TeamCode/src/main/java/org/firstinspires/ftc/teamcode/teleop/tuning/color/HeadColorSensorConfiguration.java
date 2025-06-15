package org.firstinspires.ftc.teamcode.teleop.tuning.color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;


@TeleOp(name="HEAD COLOR CONFIG")
@Config
public class HeadColorSensorConfiguration extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorRangefinder crf =
                new ColorRangefinder(hardwareMap.get(RevColorSensorV3.class, "colorSensorHead"));

        /*
        Using this example configuration, you can detect all three sample colors based on which pin is reading true:
        both      --> yellow
        only pin0 --> red
        only pin1 --> blue
        neither   --> no object
         */
        crf.setPin0Digital(ColorRangefinder.DigitalMode.HSV, 180 / 360.0 * 255, 250 / 360.0 * 255); // blue
        crf.setPin0Digital(ColorRangefinder.DigitalMode.HSV, 55 / 360.0 * 255, 90 / 360.0 * 255); // yellow
        crf.setPin0DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, 30); // 30mm or closer requirement

        crf.setPin1Digital(ColorRangefinder.DigitalMode.HSV, 0 / 360.0 * 255, 50 / 360.0 * 255); // red
        crf.setPin1Digital(ColorRangefinder.DigitalMode.HSV, 55 / 360.0 * 255, 90 / 360.0 * 255); // yellow
        crf.setPin1DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, 30); // 30mm or closer requirement

        waitForStart();

        stop();
    }
}