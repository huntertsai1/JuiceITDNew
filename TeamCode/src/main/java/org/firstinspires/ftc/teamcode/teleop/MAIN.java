package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.WinchTimeAction;
import org.firstinspires.ftc.teamcode.commands.teleop.TeleAutoCycle;
import org.firstinspires.ftc.teamcode.commands.teleop.TeleRelocToHP;
import org.firstinspires.ftc.teamcode.commands.util.CancellableAction;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.util.enums.AllianceColor;
import org.firstinspires.ftc.teamcode.util.enums.Levels;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;
import org.firstinspires.ftc.teamcode.util.hardware.GoBildaLEDIndicator;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="MAIN")
@Config
public class MAIN extends LinearOpMode {
    double oldTime = 0;
    AllianceColor allianceColor = AllianceColor.RED;
    SampleColors allianceSample = SampleColors.RED;

    // STATES
    boolean manualExtension = false;
    DRIVER_MODE driverMode = DRIVER_MODE.HUMAN;
    CancellableAction currentAutomation = null;
    int climbOverride = 3;

    boolean oldRBumper = false;
    boolean oldLBumper = false;
    boolean oldCross = false;
    boolean oldTriangle = false;
    boolean oldSquare = false;
    boolean oldSquare2 = false;
    boolean oldCircle = false;
    boolean oldDpadUp = false;
    boolean oldDpadDown = false;
    boolean oldDpadLeft = false;
    boolean oldCircle2 = false;
    boolean liftReset = false;
    boolean oldCross2 = false;
    boolean oldDpadRight = false;
    boolean oldOptions = false;
    boolean oldOptions2 = false;

    double oldTrigger = 0;
    double oldRtrigger = 0.0;
    int LIFT_INCREMENT = 20;
    float EXT_INCREMENT = 20;
    int autoWinches = 0;
    float f = 0.1f;
    boolean options = false;
    WinchTimeAction curW;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, false);
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));
        List<Action> actionsQueue = new ArrayList<>();
        TeleAutoCycle.depoTargetX = 9;
        robot.blinky.set(GoBildaLEDIndicator.Colors.BLUE, GoBildaLEDIndicator.Animation.SLOW_BLINK);
        robot.sweeper.setPosition(92);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        while (opModeInInit() && !isStopRequested()) {
            if (gamepad1.square) {
                allianceColor = AllianceColor.RED;
                allianceSample = SampleColors.RED;
                gamepad1.rumbleBlips(1);
                gamepad1.setLedColor(255, 0, 0, 500);
            } else if (gamepad1.circle) {
                allianceColor = AllianceColor.BLUE;
                allianceSample = SampleColors.BLUE;
                gamepad1.rumbleBlips(1);
                gamepad1.setLedColor(0, 0, 255, 500);
            }

            telemetry.addData("SELECTED COLOR: ", allianceColor);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            TelemetryPacket packet = new TelemetryPacket();

            if (gamepad1.right_bumper && !oldRBumper) {
                if (robot.state != Levels.INTAKE_INTERMEDIATE) {
                    actionsQueue.add(
                            robot.teleIntakePreset(true)
                    );
                } else {
                    actionsQueue.add(
                            robot.intakeDrop(allianceSample, gamepad1)
                    );
                }
            }
            oldRBumper= gamepad1.right_bumper;

            if ((gamepad1.right_trigger >= 0.3) && (oldRtrigger <= 0.3)){
                if (robot.state == Levels.HIGH_RUNG) {
                    actionsQueue.add(robot.specIntakeStop());
                    }
                else {
                    actionsQueue.add(robot.stopIntakeAction());
                }
            }
            oldRtrigger = gamepad1.right_trigger;


            if (gamepad1.cross &&!oldCross) {
                if (robot.mode == Robot.Gamepiece.SAMPLE) {
                    actionsQueue.add(robot.outtakeSample(true));
                } else if (robot.mode == Robot.Gamepiece.SPECIMEN && driverMode == DRIVER_MODE.AUTO){
                    robot.blinky.set(GoBildaLEDIndicator.Colors.RED, GoBildaLEDIndicator.Animation.BLINK);
                    currentAutomation = new TeleAutoCycle(drive, robot, gamepad1);
                    actionsQueue.add(
                            new SequentialAction(
                                    new InstantAction(() -> {
                                        driverMode = DRIVER_MODE.AUTO;
                                        if (robot.mode == Robot.Gamepiece.SAMPLE) {
                                            robot.blinky.set(GoBildaLEDIndicator.Colors.JOOS_ORANGE, GoBildaLEDIndicator.Animation.SLOW_BLINK);
                                        } else {
                                            robot.blinky.set(GoBildaLEDIndicator.Colors.VIOLET, GoBildaLEDIndicator.Animation.SLOW_BLINK);
                                        }
                                    }),
                                    currentAutomation,
                                    new InstantAction(() -> {
                                        currentAutomation = null;
//                                        driverMode = DRIVER_MODE.HUMAN;
                                    })
                            )
                    );
                }
            }
            oldCross = gamepad1.cross;

            if ((gamepad1.options && !oldOptions) || (gamepad2.options && !oldOptions2)){
                options = !options;
                if (options){
                    robot.blinky.set(GoBildaLEDIndicator.Colors.RED, GoBildaLEDIndicator.Animation.SLOW_BLINK);
                }else{
                    if (robot.mode == Robot.Gamepiece.SPECIMEN) {
                        robot.blinky.set(GoBildaLEDIndicator.Colors.VIOLET, GoBildaLEDIndicator.Animation.SLOW_BLINK);
                    } else {
                        robot.blinky.set(GoBildaLEDIndicator.Colors.BLUE, GoBildaLEDIndicator.Animation.SLOW_BLINK);
                    }
                }
                gamepad1.rumble(250);
            }
            oldOptions = gamepad1.options;
            oldOptions2 = gamepad2.options;

            if (gamepad1.triangle && !oldTriangle){
                if (robot.toggleGamepiece() == Robot.Gamepiece.SPECIMEN){
                    gamepad1.rumbleBlips(2);
                }else{
                    gamepad1.rumble(250);
                }
            }
            oldTriangle = gamepad1.triangle;

            if (options){
//                if (gamepad1.dpad_up && !oldDpadUp){
//                    autoWinches = 1;
//                    actionsQueue.add(new SequentialAction(new WinchTimeAction(robot.climbWinch, 1.75, -1, telemetry), new InstantAction(()->{autoWinches = 0;})));
//                }
//                else if (gamepad1.dpad_down && !oldDpadDown){
//                    autoWinches = 1;
//                    actionsQueue.add(new SequentialAction(new WinchTimeAction(robot.climbWinch, 4, 1, telemetry),  new InstantAction(()-> {autoWinches = 2;})));
//                }
                //if (autoWinches == 0 || autoWinches == 2) {
                    if (gamepad1.dpad_right || gamepad2.dpad_down) {
                        robot.climbWinch.setPower(-1);
                    } else if (gamepad1.dpad_left || gamepad2.dpad_up) {
                        robot.climbWinch.setPower(1);
                    }else {
                        robot.climbWinch.setPower(0);
                        if (autoWinches != 0){
//                            robot.climbWinch.setPower(0);
//                        }else{
                            robot.climbWinch.setPower(f);
                        }
                    }
                    if (gamepad2.dpad_right){
                        robot.climbWinch.servo1.setSpeed(1);
                    }else if (gamepad2.dpad_left){
                        robot.climbWinch.servo2.setSpeed(1);
                    }

                oldDpadUp = gamepad1.dpad_up;
                oldDpadDown = gamepad1.dpad_down;

                if ((gamepad1.left_bumper && !oldLBumper)) {
                    if (robot.mode == Robot.Gamepiece.SAMPLE){
                        actionsQueue.add(robot.highBasketAction());
                    }else{
                        actionsQueue.add(robot.highRung(true));
                    }
                }
                oldLBumper = gamepad1.left_bumper;

                if (gamepad1.circle && !oldCircle){
                    robot.climbWinch.setPower(f);
                    autoWinches = 1;
                }
                oldCircle = gamepad1.circle;
            }else{
                if (gamepad1.circle && !oldCircle && driverMode == DRIVER_MODE.HUMAN && robot.mode == Robot.Gamepiece.SPECIMEN) {
                    currentAutomation = new TeleRelocToHP(drive, robot);
                    robot.blinky.set(GoBildaLEDIndicator.Colors.RED, GoBildaLEDIndicator.Animation.BLINK);
                    actionsQueue.add(
                            new SequentialAction(
                                    new InstantAction(() -> driverMode = DRIVER_MODE.AUTO),
                                    currentAutomation,
                                    new InstantAction(() -> {
                                        currentAutomation = null;
//                                        driverMode = DRIVER_MODE.HUMAN;
                                    })
                            )
                    );
                }
                if (gamepad1.circle && !oldCircle && robot.mode == Robot.Gamepiece.SAMPLE){
                    actionsQueue.add(
                            new SequentialAction(
                                    new InstantAction(() -> {
                                        robot.arm.runToPreset(Levels.EJECT);
                                    }),
                                    new SleepAction(0.3),
                                    robot.claw.ejectSample(true),
                                    new SleepAction(0.2),
                                    robot.stopIntakeAction()
                            )
                    );
                }
                oldCircle = gamepad1.circle;
                if (robot.state == Levels.HIGH_BASKET && gamepad1.dpad_up && !oldDpadUp) {
                    if (robot.lift.getPos() + LIFT_INCREMENT < robot.lift.MAX){
                        robot.lift.runToPosition(robot.lift.getPos() + LIFT_INCREMENT);
                        robot.lift.HIGH_BASKET += LIFT_INCREMENT;
                    }
                } else if (robot.state == Levels.HIGH_BASKET && gamepad1.dpad_down && !oldDpadDown) {
                    robot.lift.runToPosition(robot.lift.getPos() - LIFT_INCREMENT);
                    robot.lift.HIGH_BASKET -= LIFT_INCREMENT;
                }
                oldDpadUp = gamepad1.dpad_up;
                oldDpadDown = gamepad1.dpad_down;

                if (robot.state == Levels.INTAKE_INTERMEDIATE || robot.state == Levels.INTAKE){
                    if (gamepad1.dpad_right && !oldDpadRight && robot.extension.getPosition() + EXT_INCREMENT < robot.extension.high){
                        robot.extension.runToPosition(robot.extension.getPosition() + EXT_INCREMENT);
                    }
                    else if (gamepad1.dpad_left && !oldDpadLeft && robot.extension.getPosition() -EXT_INCREMENT > robot.extension.low){
                        robot.extension.runToPosition(robot.extension.getPosition() - EXT_INCREMENT);
                    }
                }
                else {
                    if ((gamepad1.left_bumper && !oldLBumper) || (gamepad1.dpad_left && !oldDpadLeft)) {
                        if (robot.mode == Robot.Gamepiece.SAMPLE){
                            actionsQueue.add(robot.highBasketAction());
                        }else{
                            actionsQueue.add(robot.highRung(true));
                        }
                    }
                    if(gamepad1.dpad_right && !oldDpadRight){
                        actionsQueue.add(robot.outtakeSample(true));
                    }
                }
                oldLBumper = gamepad1.left_bumper;
                oldDpadRight = gamepad1.dpad_right;
                oldDpadLeft = gamepad1.dpad_left;
            }

            if (gamepad2.circle && !oldCircle2){
                robot.toggleColorSensor();

            }
            oldCircle2 = gamepad2.circle;

            if (robot.state == Levels.HIGH_RUNG && gamepad1.square && !oldSquare) {
                actionsQueue.add(robot.autoCenterSpecimen(true));
            } else if (gamepad1.square && !oldSquare){
                actionsQueue.add(
                        new ParallelAction(
                                robot.sweeper.sweep(),
                                new InstantAction(()->{robot.extension.nextShort = true;})
//                                robot.claw.ejectOps(true)
                        )
                );
            }
            oldSquare = gamepad1.square;

            if (gamepad1.left_trigger >= 0.3 && oldTrigger <= 0.3) {
                if (robot.mode == Robot.Gamepiece.SPECIMEN) {
                    actionsQueue.add(
                            new SequentialAction(
                                    new InstantAction(() -> {
                                        robot.arm.runToPreset(Levels.EJECT);
                                    }),
                                    new SleepAction(0.5),
                                    robot.claw.ejectSample(true),
                                    new SleepAction(0.5),
                                    robot.stopIntakeAction()
                            )
                    );
                } else if (robot.mode == Robot.Gamepiece.SAMPLE) {
                    actionsQueue.add(robot.lowBasketAction());
                }
            }
            oldTrigger = gamepad1.left_trigger;

            if (gamepad2.square && !oldSquare2){
                actionsQueue.add(
                        new SequentialAction(
                                new InstantAction(() -> {
                                    robot.arm.runToPreset(Levels.EJECT);
                                }),
                                new SleepAction(0.5),
                                robot.claw.ejectSample(true),
                                new SleepAction(0.5),
                                robot.stopIntakeAction()
                        )
                );
            }
            oldSquare2 = gamepad2.square;


            //lift resetting stuff

            if (gamepad2.cross && !oldCross2){
                liftReset = !liftReset;
            }
            oldCross2 = gamepad2.cross;
            if (liftReset) {
                if (gamepad2.left_trigger > 0.4) {
                    robot.lift.runToPosition(-500);
                } else if (gamepad2.right_trigger > 0.4) {
                    robot.lift.runToPosition(50);
                }
                if (gamepad2.circle && !oldCircle2) {
                    robot.lift.lift1.resetEncoder();
                    robot.lift.runToPosition(0);
                }
                oldCircle2 = gamepad2.circle;
            }

            List<Action> newActions = new ArrayList<>();
            for (Action action : actionsQueue) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            actionsQueue = newActions;

            if (driverMode == DRIVER_MODE.HUMAN) {
                double x = -gamepad1.left_stick_x;
                double y = -gamepad1.left_stick_y;
                double rx = gamepad1.right_stick_x;
                robot.setDrivePower(-x, y, rx);
            } else if (driverMode == DRIVER_MODE.AUTO) {
                //nothing for now
            }

            robot.lift.update();
            robot.blinky.update();

            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;

            telemetry.addData("MODE", robot.mode.toString());
            telemetry.addData("STATE: ", robot.state);
            telemetry.addData("Sample Head ", robot.claw.detectSampleHead());
            telemetry.addData("COLOR ENABLED", robot.activateSensor);
            telemetry.addData("LIFT POWER", robot.lift.power1);
            telemetry.addData("LIFT ", robot.lift.getPos());
            telemetry.addData("LIFT TARGET ", robot.lift.target);
            telemetry.addData("LOOPTIME: ", frequency);
            telemetry.addData("queue", actionsQueue);
            telemetry.addData("TIME TO ACTION", robot.timeToAction.time(TimeUnit.MILLISECONDS) - robot.afterAction.time(TimeUnit.MILLISECONDS));
            telemetry.update();
        }
    }
    enum DRIVER_MODE {
        HUMAN,
        AUTO
    }
}
