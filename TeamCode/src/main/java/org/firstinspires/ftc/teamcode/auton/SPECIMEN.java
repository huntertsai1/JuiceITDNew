package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.util.LoopAction;
import org.firstinspires.ftc.teamcode.commands.WinchTimeAction;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

import java.util.List;


@Autonomous(name = "SPECIMEN", group = "Autonomous")
public class SPECIMEN extends LinearOpMode {
    Robot robot;
    PinpointDrive drive;
    public void runOpMode() {
        Pose2d startPose = new Pose2d(6, -61.8, Math.toRadians(-90));
        robot = new Robot (hardwareMap, true);
        drive = new PinpointDrive(hardwareMap, startPose);

        double HPDeposit = -48; // TODO: tune these variables
        double spikeBack = -16;

        double veloLim = 50.0;
        double accelUpperLim = 50.0;
        double accelLowerLim = -30.0;

        TrajectoryActionBuilder preload = drive.actionBuilder(startPose)
                .lineToY(-31,
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim));

        TrajectoryActionBuilder allSpikes = preload.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(28, -46, Math.toRadians(-90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(34, spikeBack, Math.toRadians(-90)), Math.toRadians(90))

                .splineToLinearHeading(new Pose2d(48, spikeBack, Math.toRadians(-90)), Math.toRadians(-90))

                .setTangent(Math.toRadians(90))
                .lineToY(HPDeposit,
                        new TranslationalVelConstraint(120.0),
                        new ProfileAccelConstraint(-120.0, 120.0))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(44, spikeBack, Math.toRadians(-90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(56, spikeBack, Math.toRadians(-90)), Math.toRadians(-90))

                .setTangent(Math.toRadians(90))
                .lineToY(HPDeposit,
                        new TranslationalVelConstraint(120.0),
                        new ProfileAccelConstraint(-120.0, 120.0))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(52, spikeBack, Math.toRadians(-90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(64, spikeBack, Math.toRadians(-90)), Math.toRadians(-90))

                .setTangent(Math.toRadians(90))
                .lineToY(HPDeposit,
                        new TranslationalVelConstraint(120.0),
                        new ProfileAccelConstraint(-120.0, 120.0));

        TrajectoryActionBuilder intakeSpec2 = allSpikes.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(20, -47), Math.toRadians(-45));

        TrajectoryActionBuilder depositSpec2 = intakeSpec2.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(3, -31, Math.toRadians(-92)), Math.toRadians(90),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim));

        TrajectoryActionBuilder intakeSpec3 = depositSpec2.endTrajectory().fresh() //TODO: Same for 3,4,5 consider offsetting for accuracy
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(20, -47, Math.toRadians(-45)), Math.toRadians(0),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim));

        TrajectoryActionBuilder depositSpec3 = intakeSpec3.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(0, -29, Math.toRadians(-92)), Math.toRadians(90),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim));

        TrajectoryActionBuilder intakeSpec4 = depositSpec3.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(20, -47, Math.toRadians(-45)), Math.toRadians(0),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim));

        TrajectoryActionBuilder depositSpec4 = intakeSpec4.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-3, -29, Math.toRadians(-92)), Math.toRadians(90),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim));

        TrajectoryActionBuilder intakeSpec5 = depositSpec4.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(20, -47, Math.toRadians(-45)), Math.toRadians(0),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim));

        TrajectoryActionBuilder depositSpec5 = intakeSpec5.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-6, -29, Math.toRadians(-92)), Math.toRadians(90),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim));

        TrajectoryActionBuilder park = depositSpec5.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(40, -53, Math.toRadians(-45)), Math.toRadians(0));

        robot.initSubsystems();
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                //PRELOAD
                            new ParallelAction(
                                    preload.build(),
                                    robot.highRungAuto(true)
                            ),
                            robot.autoSpecimen(true),

                            new ParallelAction(
                                    allSpikes.build(),
                                    new SequentialAction(
                                            new SleepAction(0.2),
                                            robot.commands.preloadEjectFailSafe()
                                    )
                            ),
                            //SPEC2
                            intakeSpec2.build(),
                            robot.autoSpecIntake(true),
                            new SleepAction(2),
                                new ParallelAction(
                                    robot.highRungAuto(true),
                                    depositSpec2.build()
                            ),
                            robot.autoSpecimen(true)
//
//                            //SPEC3
//                                new ParallelAction(
//                                        intakeSpec3.build(),
//                                        robot.autoSpecIntake(true)
//                                ),
//                            new SleepAction(intakeWait),
//                                new ParallelAction(
//                                        robot.highRungAuto(true),
//                                        depositSpec3.build()
//                                ),
//                            robot.autoSpecimen(true),
//
//                            //SPEC4
//                                new ParallelAction(
//                                        intakeSpec4.build(),
//                                        robot.autoSpecIntake(true)
//                                ),
//                            new SleepAction(intakeWait),
//                                new ParallelAction(
//                                        robot.highRungAuto(true),
//                                        depositSpec4.build()
//                                ),
//                            robot.autoSpecimen(true),
//
//                            //SPEC5
//                                new ParallelAction(
//                                        intakeSpec5.build(),
//                                        robot.autoSpecIntake(true)
//                                ),
//                            new SleepAction(intakeWait),
//                                new ParallelAction(
//                                        robot.highRungAuto(true),
//                                        depositSpec5.build()
//                                ),
//                            robot.autoSpecimen(true),
//
//                            //PARK
//                            park.build()

                        ),
                        new LoopAction(() -> {
                            for (LynxModule module : allHubs) {
                                module.clearBulkCache();
                            }
                            robot.lift.update();
                            robot.blinky.update();
                        }, this::isStopRequested)
//                        , new WinchTimeAction(robot.climbWinch, 1.3, -1, telemetry)
                )
        );
    }
}
