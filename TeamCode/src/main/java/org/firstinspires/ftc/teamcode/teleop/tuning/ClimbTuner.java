package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.WinchTimeAction;
import org.firstinspires.ftc.teamcode.subsystems.climb.ContinuousServoEncoder;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp(group = "competition")
@Config
public class ClimbTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryPacket packet = new TelemetryPacket();
        ContinuousServoEncoder climb1Encoder = new ContinuousServoEncoder(hardwareMap.get(AnalogInput.class, "climb1Encoder"), true);
        ContinuousServoEncoder climb2Encoder = new ContinuousServoEncoder(hardwareMap.get(AnalogInput.class, "climb2Encoder"), false);
        Robot robot = new Robot(hardwareMap, false);
        double start1 = climb1Encoder.getPosition();
        int wraps = 0;
        double lastPosition = 0;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        List<Action> actionsQueue = new ArrayList<>();
        AtomicBoolean climbing = new AtomicBoolean(false);

        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            double climb1 = climb1Encoder.getPosition();
            double climb2 = climb2Encoder.getPosition();
            telemetry.addData("climb1", climb1);
            telemetry.addData("climb2", climb2);
            telemetry.addData("climb1 with wraps", climb1 +  + (wraps*360));
            telemetry.addData("minus start", climb1 +  + (wraps*360) - start1);
            if (!climbing.get()) {
                if (gamepad1.dpad_up) {
                    robot.climbWinch.setPower(1);
                } else if (gamepad1.dpad_down) {
                    robot.climbWinch.setPower(-1);
                } else if (gamepad1.dpad_right) {
                    robot.climbWinch.servo1.setSpeed(-1);
                } else if (gamepad1.dpad_left) {
                    robot.climbWinch.servo2.setSpeed(-1);
                } else {
                    robot.climbWinch.setPower(0);
                }
            }
            if (gamepad1.circle){
                climbing.set(true);
                actionsQueue.add(new SequentialAction(new WinchTimeAction(robot.climbWinch, 0.3, -1, telemetry), new InstantAction(()->{
                    climbing.set(false);})));
            }

            if (robot.climbWinch.getPosition() - lastPosition < -270) {
                wraps -= 1;
            }
            else if (robot.climbWinch.getPosition() - lastPosition > 270) {
                wraps += 1;
            }
            List<Action> newActions = new ArrayList<>();
            for (Action action : actionsQueue) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            actionsQueue = newActions;
            lastPosition = robot.climbWinch.getPosition();

            telemetry.update();
        }
    }
}