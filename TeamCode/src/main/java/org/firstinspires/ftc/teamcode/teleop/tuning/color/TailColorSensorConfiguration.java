package org.firstinspires.ftc.teamcode.teleop.tuning.color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TAIL COLOR CONFIG")
@Config
public class TailColorSensorConfiguration extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorRangefinder crf =
                new ColorRangefinder(hardwareMap.get(RevColorSensorV3.class, "colorSensor"));

        /*
        Using this example configuration, you can detect all three sample colors based on which pin is reading true:
        both      --> in distance
        only pin0 --> in distance
        only pin1 --> in distance
        neither   --> no object
         */
        crf.setPin0Digital(ColorRangefinder.DigitalMode.DISTANCE, 0, 20);
        crf.setPin1Digital(ColorRangefinder.DigitalMode.DISTANCE, 0, 20);

        waitForStart();

        stop();
    }
}

