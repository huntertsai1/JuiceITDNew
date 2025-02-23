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

        double depositX = -52.8;
        double depositY = -48.5;
        double depositWait = 1.2;
        double intakeWait = 0.9;

        double veloLim = 60.0;
        double accelUpperLim = 60.0;
        double accelLowerLim = -40.0;

        TrajectoryActionBuilder preload = drive.actionBuilder(startPose)
                //preload
                .setTangent(Math.toRadians(160))
                .splineToLinearHeading(new Pose2d(depositX, depositY, Math.toRadians(45)), Math.toRadians(160))
                .waitSeconds(0.8);

        TrajectoryActionBuilder spike1 = preload.endTrajectory().fresh()
                //spike1
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-45, -48, Math.toRadians(106)), Math.toRadians(45))
                .waitSeconds(intakeWait);

        TrajectoryActionBuilder deposit1 = spike1.endTrajectory().fresh()
                //depo1
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(depositX, depositY, Math.toRadians(30)), Math.toRadians(200))
                .waitSeconds(depositWait);

        TrajectoryActionBuilder spike2 = deposit1.endTrajectory().fresh()
                //spike2
                .setTangent(Math.toRadians(92))
                .splineToLinearHeading(new Pose2d(-58, -48, Math.toRadians(112)), Math.toRadians(100))
                .waitSeconds(intakeWait);

        TrajectoryActionBuilder deposit2 = spike2.endTrajectory().fresh()
                //depo2
                .setTangent(Math.toRadians(272))
                .splineToLinearHeading(new Pose2d(depositX, depositY, Math.toRadians(40)), Math.toRadians(272))
                .waitSeconds(depositWait);

        TrajectoryActionBuilder spike3 = deposit2.endTrajectory().fresh()
                //spike3
                .setTangent(Math.toRadians(88))
                .splineToLinearHeading(new Pose2d(-64.5, -45, Math.toRadians(119)), Math.toRadians(95))
                .waitSeconds(intakeWait);

        TrajectoryActionBuilder deposit3 = spike3.endTrajectory().fresh()
                //depo3
                .setTangent(Math.toRadians(268))
                .splineToLinearHeading(new Pose2d(depositX, depositY, Math.toRadians(40)), Math.toRadians(268))
                .waitSeconds(depositWait);

        TrajectoryActionBuilder subDrive = deposit3.endTrajectory().fresh()
                //ascent zone park
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-48, -11, Math.toRadians(0)), Math.toRadians(0))
                .lineToX(-20);

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
                                new SleepAction(1),

                                new InstantAction(() -> robot.claw.setPower(0)),
                                new ParallelAction(
                                        robot.autoHighBasketAction(),
                                        deposit1.build()
                                ),
                                robot.outtakeSample(true),

                                spike2.build(),

                                robot.autoBucketIntake(true),
                                new SleepAction(1),

                                new InstantAction(() -> robot.claw.setPower(0)),
                                new ParallelAction(
                                        robot.autoHighBasketAction(),
                                        deposit2.build()
                                ),
                                robot.outtakeSample(true),

                                spike3.build(),

                                robot.autoBucketIntakeTHIRD(true),
                                new SleepAction(1),

                                new InstantAction(() -> robot.claw.setPower(0)),
                                new ParallelAction(
                                        robot.autoHighBasketAction(),
                                        deposit3.build()
                                ),
                                robot.outtakeSample(true),
                                new SleepAction(1),
                                subDrive.build(),
                                robot.sweeper.sweep(),
                                new SleepAction(1),
                                robot.sweeper.sweep()
                                ),
                        new LoopAction(() -> {
                            robot.lift.update();
                        }, this::isStopRequested)
                        , new WinchTimeAction(robot.climbWinch, 2.2, -1, telemetry)
                )
        );
    }
}
