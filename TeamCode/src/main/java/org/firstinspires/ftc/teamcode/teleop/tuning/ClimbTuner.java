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
import org.firstinspires.ftc.teamcode.commands.WinchAlign;
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
        Robot robot = new Robot(hardwareMap, false);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        List<Action> actionsQueue = new ArrayList<>();
        AtomicBoolean climbing = new AtomicBoolean(false);
        boolean oldCircle = false;
        boolean oldTriangle = false;
        // Initialize your own robot class
        robot.climbWinch.equalServosStart();
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {

            double climb1W = robot.climbWinch.getPosition1();
            double climb2W = robot.climbWinch.getPosition2();
            telemetry.addData("climb1W", climb1W);
            telemetry.addData("climb2W", climb2W);
            telemetry.addData("climb1Wwraps", robot.climbWinch.wraps1);
            telemetry.addData("climb2Wwraps", robot.climbWinch.wraps2);
            telemetry.addData("climb1", robot.climbWinch.servo1.getAngle());
            telemetry.addData("climb2", robot.climbWinch.servo2.getAngle());

//            telemetry.addData("climb1", robot.climbWinch.servo1.encoder.encoder.getVoltage());
//            telemetry.addData("climb2", robot.climbWinch.servo2.encoder.encoder.getVoltage());
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
            if (gamepad1.circle && !oldCircle){
                climbing.set(true);
                actionsQueue.add(new SequentialAction(new WinchTimeAction(robot.climbWinch, 0.3, -1, telemetry), new InstantAction(()->{
                    climbing.set(false);})));
            }
            oldCircle = gamepad1.circle;

            if (gamepad1.triangle&& !oldTriangle){
                climbing.set(true);
                actionsQueue.add(new SequentialAction(new WinchAlign(robot.climbWinch, telemetry), new InstantAction(()->{
                    climbing.set(false);})));
            }
            oldTriangle = gamepad1.triangle;

            List<Action> newActions = new ArrayList<>();
            for (Action action : actionsQueue) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            actionsQueue = newActions;

            robot.climbWinch.update();
            telemetry.update();
        }
    }
}