package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Pose2d;
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
        double spikeBack = -13;

        Action preload = drive.actionBuilder(startPose)
                .lineToY(-33)
                .waitSeconds(1)
                .build();

        Action allSpikes = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(38, -33, Math.toRadians(-90)), Math.toRadians(0))
                .setTangent(Math.toRadians(90))
                .lineToY(spikeBack)
                .setTangent(Math.toRadians(0))
                .lineToX(44)
                .setTangent(Math.toRadians(90))
                .lineToY(HPDeposit)
                .setTangent(Math.toRadians(0))
                .lineToX(49)
                .setTangent(Math.toRadians(90))
                .lineToY(spikeBack)
                .setTangent(Math.toRadians(0))
                .lineToX(54)
                .setTangent(Math.toRadians(90))
                .lineToY(HPDeposit)
                .setTangent(Math.toRadians(0))
                .lineToX(59)
                .setTangent(Math.toRadians(90))
                .lineToY(spikeBack)
                .setTangent(Math.toRadians(0))
                .lineToX(63.5)
                .setTangent(Math.toRadians(90))
                .lineToY(HPDeposit)
                .build();

        Action cycle1 = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(24, -44, Math.toRadians(-45)), Math.toRadians(180))
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(4, -33), Math.toRadians(135))
                .waitSeconds(0.5)
                .build();

        robot.initSubsystems();

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                            preload,
                            allSpikes,
                            cycle1
                        ),
                        new LoopAction(() -> {
//                            robot.lift.update();
                        }, this::isStopRequested)
                )
        );
    }
}
