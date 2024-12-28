package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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

        double HPDeposit = -46;
        double spikeBack = -12;

        TrajectoryActionBuilder preload = drive.actionBuilder(startPose)
                .lineToY(-33)
                .waitSeconds(0.2);

        TrajectoryActionBuilder allSpikes = preload.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(30, -38, Math.toRadians(-90)), Math.toRadians(0))
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
                .lineToX(64)
                .setTangent(Math.toRadians(90))
                .lineToY(HPDeposit);
        
        TrajectoryActionBuilder cycle1 = allSpikes.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(16, -44, Math.toRadians(-45)), Math.toRadians(180))
                .waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(6, -31, Math.toRadians(-97)), Math.toRadians(90))
                .waitSeconds(0.2);

        robot.initSubsystems();

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                            preload.build(),
                            allSpikes.build(),
                            cycle1.build()
                        ),
                        new LoopAction(() -> {
//                            robot.lift.update();
                        }, this::isStopRequested)
                )
        );
    }
}
