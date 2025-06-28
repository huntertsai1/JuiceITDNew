package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.util.LoopAction;
import org.firstinspires.ftc.teamcode.commands.WinchTimeAction;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;

import java.util.List;


@Autonomous(name = "SAMPLE", group = "Autonomous")
public class SAMPLE extends LinearOpMode {
    Robot robot;
    PinpointDrive drive;
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-38, -62, Math.toRadians(0));
        robot = new Robot (hardwareMap, true);
        drive = new PinpointDrive(hardwareMap, startPose);

        double depositX = -53;
        double depositY = -51;
        double depositWait = 1.2;
        double intakeWait = 0.9;

        double veloLim = 6.0;
        double accelUpperLim = 6.0;
        double accelLowerLim = -6.0;

        TrajectoryActionBuilder preload = drive.actionBuilder(startPose)
                //preload
                .setTangent(Math.toRadians(160))
                .splineToLinearHeading(new Pose2d(depositX, depositY, Math.toRadians(45)), Math.toRadians(160))
                .waitSeconds(0.5);

        TrajectoryActionBuilder spike1 = preload.endTrajectory().fresh()
                //spike1
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-47.7, -50, Math.toRadians(91)), Math.toRadians(45));

        TrajectoryActionBuilder spike1feed = spike1.endTrajectory().fresh()
                .setTangent(Math.toRadians(91))
                .lineToY(-38,
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim));

        TrajectoryActionBuilder deposit1 = spike1feed.endTrajectory().fresh()
                //depo1
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(-50, -52, Math.toRadians(30)), Math.toRadians(200))
                .waitSeconds(0.5);

        TrajectoryActionBuilder spike2 = deposit1.endTrajectory().fresh()
                //spike2
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-57.7, -48, Math.toRadians(91)), Math.toRadians(90));

        TrajectoryActionBuilder spike2feed = spike2.endTrajectory().fresh()
                .setTangent(Math.toRadians(91))
                .lineToY(-36,
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim));

        TrajectoryActionBuilder deposit2 = spike2feed.endTrajectory().fresh()
                //depo2
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-52, -51, Math.toRadians(45)), Math.toRadians(270))
                .waitSeconds(0.5);

        TrajectoryActionBuilder spike3 = deposit2.endTrajectory().fresh()
                //spike3
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-60, -48, Math.toRadians(107)), Math.toRadians(90));

        TrajectoryActionBuilder spike3feed = spike3.endTrajectory().fresh()
                .setTangent(Math.toRadians(100))
                .lineToY(-35,
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim));

        TrajectoryActionBuilder deposit3 = spike3feed.endTrajectory().fresh()
                //depo3
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-52, -51, Math.toRadians(45)), Math.toRadians(270))
                .waitSeconds(0.5);

        TrajectoryActionBuilder subDrive = deposit3.endTrajectory().fresh()
                //ascent zone park
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-40, -8, Math.toRadians(0)), Math.toRadians(0))
                .lineToX(-20);

        robot.bucketinitSubsystems();
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        new InstantAction(() -> robot.claw.setPower(0)),
                        new SequentialAction(
                                new ParallelAction(
                                        preload.build(),
                                        robot.autoHighBasketAction()
                                ),
                                robot.outtakeSample(true),
                                new SleepAction(0.5),
                                new ParallelAction(
                                        robot.autoBucketIntake(true),
                                        spike1.build()
                                ),
                                new ParallelAction(
                                        spike1feed.build(),
                                        robot.commands.stopIntakeTimeout(3, SampleColors.YELLOW, SampleColors.BLUE, SampleColors.RED)
                                ),
                                new ParallelAction(
                                        robot.autoHighBasketAction(),
                                        deposit1.build()
                                ),
                                robot.outtakeSample(true),
                                new SleepAction(0.5),
                                new ParallelAction(
                                        robot.autoBucketIntake(true),
                                        spike2.build()
                                ),
                                new ParallelAction(
                                        spike2feed.build(),
                                        robot.commands.stopIntakeTimeout(3, SampleColors.YELLOW, SampleColors.BLUE, SampleColors.RED)
                                ),
                                new ParallelAction(
                                        robot.autoHighBasketAction(),
                                        deposit2.build()
                                ),
                                robot.outtakeSample(true),
                                new SleepAction(0.5),
                                new ParallelAction(
                                        robot.autoBucketIntake(true),
                                        spike3.build()
                                ),
                                new ParallelAction(
                                        spike3feed.build(),
                                        robot.commands.stopIntakeTimeout(3, SampleColors.YELLOW, SampleColors.BLUE, SampleColors.RED)
                                ),
                                new ParallelAction(
                                        robot.autoHighBasketAction(),
                                        deposit3.build()
                                ),
                                robot.outtakeSample(true),
                                new SleepAction(0.5),
                                subDrive.build(),
                                robot.sweeper.sweep(),
                                new SleepAction(0.5),
                                robot.sweeper.sweep()
                        ),
                        new LoopAction(() -> {
                            for (LynxModule module : allHubs) {
                                module.clearBulkCache();
                            }
                            robot.lift.update();
                            robot.blinky.update();
                        }, this::isStopRequested)
//                        , new WinchTimeAction(robot.climbWinch, 2.2, -1, telemetry)
                )
        );
    }
}