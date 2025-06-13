package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.WinchTimeAction;
import org.firstinspires.ftc.teamcode.commands.teleop.TeleAutoCycle;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.util.enums.Levels;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;
import org.firstinspires.ftc.teamcode.util.hardware.Motor;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="PRE")
@Config
public class PreMatch extends LinearOpMode {
    double oldTime = 0;

    boolean oldCross = false;
    boolean oldDpadL = false;
    boolean oldDpadR = false;
    WinchTimeAction curW;
    @Override
    public void runOpMode() throws InterruptedException {
//        Motor backLeft = new Motor(0, "leftBack", hardwareMap, true);
//        Motor backRight = new Motor(1, "rightBack", hardwareMap, false);
//        Motor frontLeft = new Motor(2, "leftFront", hardwareMap, true);
//        Motor frontRight = new Motor(3, "rightFront", hardwareMap, false);
        Robot robot = new Robot(hardwareMap, false);
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));
        List<Action> actionsQueue = new ArrayList<>();
        Action[][] actions = { //(start, stop)
            {new ParallelAction(
                robot.sweeper.sweep(),
                robot.claw.ejectOps(true)
            ),new SleepAction(0.1)},
            {new WinchTimeAction(robot.climbWinch, 1.75, -1, telemetry), new WinchTimeAction(robot.climbWinch, 1.75, -1, telemetry)},
            {new SequentialAction(robot.teleIntakePreset(true), robot.intakeDrop(SampleColors.RED)), robot.stopIntakeAction()},
            {robot.highBasketAction(), robot.outtakeSample(true)},
            {new InstantAction(()->{robot.backLeft.setSpeed(1);}),new InstantAction(()->{robot.backLeft.setSpeed(0);})},
            {new InstantAction(()->{robot.backRight.setSpeed(1);}),new InstantAction(()->{robot.backRight.setSpeed(0);})},
            {new InstantAction(()->{robot.frontLeft.setSpeed(1);}),new InstantAction(()->{robot.frontLeft.setSpeed(0);})},
            {new InstantAction(()->{robot.frontRight.setSpeed(1);}),new InstantAction(()->{robot.frontRight.setSpeed(1);})},
//            {new InstantAction(()->{backLeft.setSpeed(1);}), new InstantAction(()->{backLeft.setSpeed(0);})},
//            {new InstantAction(()->{backRight.setSpeed(1);}),new InstantAction(()->{backRight.setSpeed(0);})},
//            {new InstantAction(()->{frontLeft.setSpeed(1);}),new InstantAction(()->{frontLeft.setSpeed(0);})},
//            {new InstantAction(()->{frontRight.setSpeed(1);}),new InstantAction(()->{frontRight.setSpeed(0);})}
        };

        waitForStart();
        if (isStopRequested()) return;
        int i = 0;
        boolean run = false;
        boolean which = true;
        while (opModeIsActive() && !isStopRequested() && i<actions.length) {
            TelemetryPacket packet = new TelemetryPacket();
            if (gamepad1.cross && !oldCross) {
               which = !which;
               run = true;
            }
            oldCross = gamepad1.cross;

            if (run) {
                if (which) {
                    actionsQueue.add(actions[i][0]);

                } else {
                    actionsQueue.add(actions[i][1]);
                }
                run = false;
            }
            if (gamepad1.dpad_left && !oldDpadL){
                i--;
                which = false;
            }
            oldDpadL = gamepad1.dpad_left;

            if (gamepad1.dpad_right && !oldDpadR){
                i++;
                which = false;
            }
            oldDpadR = gamepad1.dpad_right;

            List<Action> newActions = new ArrayList<>();
            for (Action action : actionsQueue) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            actionsQueue = newActions;

            //robot.lift.update();


            telemetry.addData("MODE",i);
            telemetry.addData("run", run);
            telemetry.addData("which", which);
            //telemetry.addData("STATE: ", robot.state);
            telemetry.update();
        }
    }
}
