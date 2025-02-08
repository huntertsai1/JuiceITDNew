package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LoopAction;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

@Disabled
@Autonomous(name = "BucketSide", group = "Autonomous")
public class BucketSide extends LinearOpMode {
    Robot robot;
    PinpointDrive drive;
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-38, -62, Math.toRadians(0));
        robot = new Robot(hardwareMap, true);
        drive = new PinpointDrive(hardwareMap, startPose);

        double intakeWait = 0.3;

        TrajectoryActionBuilder preload = drive.actionBuilder(startPose)
                //preload
                .setTangent(Math.toRadians(160))
                .splineToLinearHeading(new Pose2d(-53.5, -53.5, Math.toRadians(45)), Math.toRadians(160))
                .waitSeconds(1.2);

        TrajectoryActionBuilder spike1 = preload.endTrajectory().fresh()
                //spike1
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(97)), Math.toRadians(45))
                .waitSeconds(0.5);

        TrajectoryActionBuilder depo1 = spike1.endTrajectory().fresh()
                //depo1
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(-53.5, -53.5, Math.toRadians(40)), Math.toRadians(200))
                .waitSeconds(1.2);

        TrajectoryActionBuilder spike2 = depo1.endTrajectory().fresh()
                //spike2
                .setTangent(Math.toRadians(92))
                .splineToLinearHeading(new Pose2d(-58, -48, Math.toRadians(99)), Math.toRadians(100))
                .waitSeconds(0.5);

        TrajectoryActionBuilder depo2 = spike2.endTrajectory().fresh()
                //depo2
                .setTangent(Math.toRadians(272))
                .splineToLinearHeading(new Pose2d(-53.5, -53.5, Math.toRadians(45)), Math.toRadians(272))
                .waitSeconds(1.2);

        TrajectoryActionBuilder spike3 = depo2.endTrajectory().fresh()
                //spike3
                .setTangent(Math.toRadians(88))
                .splineToLinearHeading(new Pose2d(-62, -47, Math.toRadians(107)), Math.toRadians(95))
                .waitSeconds(0.5);

        TrajectoryActionBuilder depo3 = spike3.endTrajectory().fresh()
                //depo3
                .setTangent(Math.toRadians(268))
                .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.toRadians(268))
                .waitSeconds(1.2);

        TrajectoryActionBuilder subDrive = depo3.endTrajectory().fresh()
                //ascent zone park
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-31, -7, Math.toRadians(0)), Math.toRadians(0));

        TrajectoryActionBuilder depo4 = subDrive.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-51, -51, Math.toRadians(45)), Math.toRadians(268))
                .waitSeconds(1.2);

        telemetry.addData("is","starting");
        telemetry.update();

        robot.initSubsystems();
        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                    new InstantAction(() -> robot.claw.setPower(0)),
                    new SequentialAction(
                        new ParallelAction(
                            robot.autoHighBasketAction(),
                            preload.build()
                        ),

                        robot.outtakeSample(true),

                        spike1.build(),
                        robot.autoIntake(true),
                        new SleepAction(0.5),

                        new InstantAction(() -> robot.claw.setPower(0)),
                        new ParallelAction(
                            robot.autoHighBasketAction(),
                            depo1.build()
                        ),

                        robot.outtakeSample(true),

                        spike2.build(),
                        robot.autoIntake(true),
                        new SleepAction(0.5),

                        new InstantAction(() -> robot.claw.setPower(0)),
                        new ParallelAction(
                            robot.autoHighBasketAction(),
                            depo2.build()
                        ),

                        robot.outtakeSample(true),

                        spike3.build(),
                        robot.autoIntake(true),
                        new SleepAction(0.5),

                        new InstantAction(() -> robot.claw.setPower(0)),
                        new ParallelAction(
                            robot.autoHighBasketAction(),
                            depo3.build()
                        ),

                        robot.outtakeSample(true),

                        subDrive.build(),
                        robot.autoIntake(true),
                        new SleepAction(0.5),
                        robot.stopIntakeAction(),

                        new InstantAction(() -> robot.claw.setPower(0)),
                        new ParallelAction(
                            depo4.build(),
                            new SequentialAction(
                                new SleepAction(0.1),
                                robot.autoHighBasketAction()
                            )
                        ),

                        robot.outtakeSample(true)
                    ),
                        new LoopAction(() -> {
                            robot.lift.update();
                        }, this::isStopRequested)
            ));
    }
}