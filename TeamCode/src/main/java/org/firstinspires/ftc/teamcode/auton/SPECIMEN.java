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

        double HPDeposit = -48;
        double spikeBack = -12;
        double waits = 0.2;
        double intakeWait = 1;

        double veloLim = 60.0;
        double accelUpperLim = 60.0;
        double accelLowerLim = -40.0;

        TrajectoryActionBuilder preload = drive.actionBuilder(startPose)
                .lineToY(-33,
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim));

        TrajectoryActionBuilder allSpikes = preload.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(30, -42, Math.toRadians(-90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(40, -12, Math.toRadians(-90)), Math.toRadians(90))
                .setTangent(Math.toRadians(0))
                .lineToX(44)
                .setTangent(Math.toRadians(90))
                .lineToY(HPDeposit)
                .lineToY(spikeBack)
                .setTangent(Math.toRadians(0))
                .lineToX(54)
                .setTangent(Math.toRadians(90))
                .lineToY(HPDeposit)
                .lineToY(spikeBack)
                .setTangent(Math.toRadians(0))
                .lineToX(63)
                .setTangent(Math.toRadians(90))
                .lineToY(HPDeposit);
        
        TrajectoryActionBuilder intakeSpec2 = allSpikes.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(17, -46), Math.toRadians(-45));

        TrajectoryActionBuilder depositSpec2 = intakeSpec2.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(6, -33, Math.toRadians(-92)), Math.toRadians(90),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim));

        TrajectoryActionBuilder intakeSpec3 = intakeSpec2.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(17, -46, Math.toRadians(-45)), Math.toRadians(0),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim));

        TrajectoryActionBuilder depositSpec3 = intakeSpec3.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(6, -33, Math.toRadians(-92)), Math.toRadians(90),
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

                            intakeSpec2.build(),

                            depositSpec2.build(),

                            intakeSpec3.build(),

                            depositSpec3.build()
                        ),
                        new LoopAction(() -> {
                            robot.lift.update();
                        }, this::isStopRequested)
                )
        );
    }
}
