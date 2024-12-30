package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LoopAction;
import org.firstinspires.ftc.teamcode.util.enums.Levels;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.util.enums.Levels;


@Autonomous(name = "SPECIMEN", group = "Autonomous")
public class SPECIMEN extends LinearOpMode {
    Robot robot;
    PinpointDrive drive;
    public void runOpMode() {
        Pose2d startPose = new Pose2d(6, -62, Math.toRadians(-90));
        robot = new Robot (hardwareMap, true);
        drive = new PinpointDrive(hardwareMap, startPose);

        double HPDeposit = -52;
        double spikeBack = -12;
        double waits = 0.2;
        double intakeWait = 1;

        double veloLim = 60.0;
        double accelUpperLim = 60.0;
        double accelLowerLim = -40.0;

        TrajectoryActionBuilder preload = drive.actionBuilder(startPose)
                .lineToY(-30,
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim));

        TrajectoryActionBuilder allSpikes = preload.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(18, -42, Math.toRadians(-90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(36, spikeBack, Math.toRadians(-90)), Math.toRadians(90))

                .splineToLinearHeading(new Pose2d(46, spikeBack, Math.toRadians(-90)), Math.toRadians(-90))

                .setTangent(Math.toRadians(90))
                .lineToY(HPDeposit)

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(46, spikeBack, Math.toRadians(-90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(55, spikeBack, Math.toRadians(-90)), Math.toRadians(-90))

                .setTangent(Math.toRadians(90))
                .lineToY(HPDeposit)

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(55, spikeBack, Math.toRadians(-90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(61.5, spikeBack, Math.toRadians(-90)), Math.toRadians(-90))

                .setTangent(Math.toRadians(90))
                .lineToY(HPDeposit);

        TrajectoryActionBuilder intakeSpec2 = allSpikes.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(19, -48), Math.toRadians(-45));

        TrajectoryActionBuilder depositSpec2 = intakeSpec2.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(3, -30, Math.toRadians(-92)), Math.toRadians(90),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim));

        TrajectoryActionBuilder intakeSpec3 = depositSpec2.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(19, -48, Math.toRadians(-45)), Math.toRadians(0),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim));

        TrajectoryActionBuilder depositSpec3 = intakeSpec3.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(0, -30, Math.toRadians(-92)), Math.toRadians(90),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim));

        TrajectoryActionBuilder intakeSpec4 = depositSpec3.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(19, -48, Math.toRadians(-45)), Math.toRadians(0),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim));

        TrajectoryActionBuilder depositSpec4 = intakeSpec4.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-3, -30, Math.toRadians(-92)), Math.toRadians(90),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim));

        TrajectoryActionBuilder intakeSpec5 = depositSpec4.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(19, -48, Math.toRadians(-45)), Math.toRadians(0),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim));

        TrajectoryActionBuilder depositSpec5 = intakeSpec5.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-6, -30, Math.toRadians(-92)), Math.toRadians(90),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim));

        robot.initSubsystems();

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                //PRELOAD
                                new ParallelAction(
                                        preload.build(),
                                        robot.highRung(true)
                                ),
                            robot.autoSpecimen(true),

                            allSpikes.build(),

                            //SPEC2

                            intakeSpec2.build(),
                            robot.autoIntake(true),
                            new SleepAction(0.5),
                            robot.highRung(true),
                            new SleepAction(waits),

                            depositSpec2.build(),
                            robot.autoSpecimen(true),

                            //SPEC3
                                new ParallelAction(
                                        intakeSpec3.build(),
                                        robot.autoIntake(true)
                                ),
                            new SleepAction(waits),
                                robot.highRung(true),
                            new SleepAction(waits),

                            depositSpec3.build(),
                            robot.autoSpecimen(true),

                            //SPEC4
                                new ParallelAction(
                                        intakeSpec4.build(),
                                        robot.autoIntake(true)
                                ),
                            new SleepAction(waits),
                            robot.highRung(true),
                            new SleepAction(waits),

                            depositSpec4.build(),
                            robot.autoSpecimen(true),

                            //SPEC5
                                new ParallelAction(
                                        intakeSpec5.build(),
                                        robot.autoIntake(true)
                                ),
                            new SleepAction(waits),
                            robot.highRung(true),
                            new SleepAction(waits),

                            depositSpec5.build(),
                            robot.autoSpecimen(true)

                        ),
                        new LoopAction(() -> {
                            robot.lift.update();
                        }, this::isStopRequested)
                )
        );
    }
}
