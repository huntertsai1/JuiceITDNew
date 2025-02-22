package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.util.LoopAction;
import org.firstinspires.ftc.teamcode.commands.WinchTimeAction;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;


@Autonomous(name = "BUCKET", group = "Autonomous")
public class BETTER_BUCKET extends LinearOpMode {
    Robot robot;
    PinpointDrive drive;
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-38, -62, Math.toRadians(0));
        robot = new Robot (hardwareMap, true);
        drive = new PinpointDrive(hardwareMap, startPose);

        double HPDeposit = -51;
        double spikeBack = -16.5;
        double waits = 0.2;
        double intakeWait = 0.3;

        double veloLim = 60.0;
        double accelUpperLim = 60.0;
        double accelLowerLim = -40.0;

        TrajectoryActionBuilder preload = drive.actionBuilder(startPose)
                //preload
                .setTangent(Math.toRadians(160))
                .splineToLinearHeading(new Pose2d(-54, -51, Math.toRadians(45)), Math.toRadians(160))
                .waitSeconds(0.4);

        TrajectoryActionBuilder spike1 = preload.endTrajectory().fresh()
                //spike1
                .turn(Math.toRadians(41))
                .waitSeconds(0.2);

        TrajectoryActionBuilder deposit1 = spike1.endTrajectory().fresh()
                //deposit1
                .turn(Math.toRadians(-45))
                .waitSeconds(0.3);

        TrajectoryActionBuilder spike2 = deposit1.endTrajectory().fresh()
                //spike2
                .turn(Math.toRadians(65))
                .waitSeconds(0.2);

        TrajectoryActionBuilder deposit2 = spike2.endTrajectory().fresh()
                //deposit2
                .turn(Math.toRadians(-66))
                .waitSeconds(0.3);

        TrajectoryActionBuilder spike3 = deposit2.endTrajectory().fresh()
                //spike3
                .setTangent(Math.toRadians(88))
                .splineToLinearHeading(new Pose2d(-62, -47, Math.toRadians(115)), Math.toRadians(95))
                .waitSeconds(0.5);

        TrajectoryActionBuilder deposit3 = spike3.endTrajectory().fresh()
                //deposit3
                .setTangent(Math.toRadians(268))
                .splineToLinearHeading(new Pose2d(-55, -48, Math.toRadians(45)), Math.toRadians(268))
                .waitSeconds(1.2);

                TrajectoryActionBuilder subDrive = deposit3.endTrajectory().fresh()
                //ascent zone park
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-31, -7, Math.toRadians(0)), Math.toRadians(0));

        robot.initSubsystems();

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

                                spike1.build(),

                                robot.autoBucketIntake(true),
                                new SleepAction(0.3),

                                new InstantAction(() -> robot.claw.setPower(0)),
                                new ParallelAction(
                                        robot.autoHighBasketAction(),
                                        deposit1.build()
                                ),
                                robot.outtakeSample(true),

                                spike2.build(),

                                robot.autoBucketIntake(true),
                                new SleepAction(0.3),

                                new InstantAction(() -> robot.claw.setPower(0)),
                                new ParallelAction(
                                        robot.autoHighBasketAction(),
                                        deposit2.build()
                                ),
                                robot.outtakeSample(true),

                                spike3.build(),

                                robot.autoBucketIntake(true),
                                new SleepAction(0.3),

                                new InstantAction(() -> robot.claw.setPower(0)),
                                new ParallelAction(
                                        robot.autoHighBasketAction(),
                                        deposit3.build()
                                ),
                                robot.outtakeSample(true),
                                new SleepAction(1),
                                subDrive.build()
                        ),
                        new LoopAction(() -> {
                            robot.lift.update();
                        }, this::isStopRequested)
//                        , new WinchTimeAction(robot.climbWinch, 1.24, -1, telemetry)
                )
        );
    }
}
