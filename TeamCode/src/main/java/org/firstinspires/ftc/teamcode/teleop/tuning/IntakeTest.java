package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.enums.SampleColors;
import org.firstinspires.ftc.teamcode.util.hardware.BrushlandColorSensor;
import org.firstinspires.ftc.teamcode.util.hardware.ContinuousServo;
import org.firstinspires.ftc.teamcode.util.hardware.StepperServo;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(group = "competition")
@Config
@Disabled
public class IntakeTest extends LinearOpMode {
    StepperServo ext1;
    StepperServo ext2;
    StepperServo arm;
    StepperServo elbow;
    ContinuousServo intake1;
    ContinuousServo intake2;
    BrushlandColorSensor sensorHead;
    BrushlandColorSensor sensorTail;

    public static double stopSpeed = 0.25;


    @Override
    public void runOpMode() throws InterruptedException {
        ext1 = new StepperServo(0, "ext1", hardwareMap);
        ext2 = new StepperServo(0, "ext2", hardwareMap);
        arm = new StepperServo(0, "arm", hardwareMap);
        elbow = new StepperServo(0, "elbow", hardwareMap);
        intake1 = new ContinuousServo(0, "claw1", hardwareMap);
        intake2 = new ContinuousServo(0, "claw2", hardwareMap);
        sensorHead = new BrushlandColorSensor(0, "colorSensorHead", hardwareMap);
        sensorTail = new BrushlandColorSensor(0, "colorSensorTail", hardwareMap);

        ext1.servo.setDirection(Servo.Direction.REVERSE);
        ext2.servo.setDirection(Servo.Direction.REVERSE);
        intake1.servo.setDirection(DcMotorSimple.Direction.REVERSE);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        boolean oldRBumper = false;
        boolean oldLBumper = false;

        /**
         * 0 -> intaking, -1 -> primed for capture, 1 -> confirmed capture
         */
        int intakeStatus = 0;
        ElapsedTime primeTimeout = new ElapsedTime();

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            if (gamepad1.right_bumper && !oldRBumper) {
                ext1.setAngle((float) 285);
                ext2.setAngle((float) 285);
                arm.setAngle((float) 170);
                elbow.setAngle((float) 205);
                intake1.setSpeed((float) 1);
                intake2.setSpeed((float) 1);
                intakeStatus = 0;
            }

            if (gamepad1.left_bumper && !oldLBumper) {
                intake1.setSpeed((float) -1);
                intake2.setSpeed((float) -1);
                intakeStatus = 0;
            }

            // YELLOW OR RED, check for only p1 for blue and yellow
            if (sensorHead.getPin0Digital() && intakeStatus == 0) {
                intake1.setSpeed((float) stopSpeed);
                intake2.setSpeed((float) stopSpeed);

                primeTimeout.reset();
                intakeStatus = -1;
            } else if (intakeStatus == -1 && sensorTail.getPin0Digital()) {
                intake1.setSpeed((float) 0);
                intake2.setSpeed((float) 0);
                intakeStatus = 1;
            } else if (!sensorHead.getPin0Digital() && intakeStatus == -1 && primeTimeout.time(TimeUnit.MILLISECONDS) > 750) {
                intake1.setSpeed((float) 1);
                intake2.setSpeed((float) 1);
                intakeStatus = 0;
            }

            oldRBumper = gamepad1.right_bumper;
            oldLBumper = gamepad1.left_bumper;

            boolean p0 = sensorHead.getPin0Digital();
            boolean p1 = sensorHead.getPin1();
            SampleColors s = null;

            if (p0 && p1) {
                s = SampleColors.YELLOW;
            } else if (p0 && !p1) {
                s = SampleColors.RED;
            } else if (!p0 && p1) {
                s = SampleColors.BLUE;
            }

            telemetry.addData("color", s);

            System.out.println(sensorHead.getPin0Digital() + ", " + sensorHead.getPin1() + ";   " + sensorTail.getPin0Digital() + ", " + sensorTail.getPin1() + ";   "+ intakeStatus);

            telemetry.update();

        }
    }
}
