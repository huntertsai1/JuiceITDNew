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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.util.LoopAction;
import org.firstinspires.ftc.teamcode.commands.WinchTimeAction;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

import java.util.List;


@Autonomous(name = "BUCKET", group = "Autonomous")
@Disabled
public class BETTER_BUCKET extends LinearOpMode {
    Robot robot;
    PinpointDrive drive;
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-38, -62, Math.toRadians(90));
        robot = new Robot (hardwareMap, true);
        drive = new PinpointDrive(hardwareMap, startPose);

        double depositX = -52.8;
        double depositY = -48.5;
        double depositWait = 1.2;
        double intakeWait = 0.9;

        double subSampleX = -6;
        double subSampleY = -12;

        double veloLim = 60.0;
        double accelUpperLim = 60.0;
        double accelLowerLim = -40.0;

        boolean oldDpadUp = false;
        boolean oldDpadDown = false;
        boolean oldDpadLeft = false;
        boolean oldDpadRight = false;
        boolean oldCircle = false;

        TrajectoryActionBuilder preload = drive.actionBuilder(startPose)
        //preload
            .setTangent(Math.toRadians(160))
            .splineToLinearHeading(new Pose2d(depositX, depositY, Math.toRadians(60)), Math.toRadians(90))
            .waitSeconds(depositWait);
            //spike1 = preload.endTrajectory().fresh()
            //spike1
        TrajectoryActionBuilder intake1 = preload.endTrajectory().fresh()
            .lineToY(-42, new TranslationalVelConstraint(20));

        TrajectoryActionBuilder spike2 = intake1.endTrajectory().fresh()
            //spike2
            .setTangent(Math.toRadians(135))
            .splineToLinearHeading(new Pose2d(-60, depositY, Math.toRadians(85)), Math.toRadians(135))
            .setTangent(Math.toRadians(85));
        TrajectoryActionBuilder intake2 = spike2.endTrajectory().fresh()
            .lineToY(-42, new TranslationalVelConstraint(20))
            .waitSeconds(intakeWait + depositWait);

        TrajectoryActionBuilder spike3 = intake2.endTrajectory().fresh()
                //spike3
            .setTangent(Math.toRadians(150))
            .splineToLinearHeading(new Pose2d(-62, -48, Math.toRadians(100)), Math.toRadians(150))
            .setTangent(Math.toRadians(100));
        TrajectoryActionBuilder intake3 = spike3.endTrajectory().fresh()
            .lineToY(-40, new TranslationalVelConstraint(20))
            .waitSeconds(intakeWait);


        TrajectoryActionBuilder deposit3 = intake3.endTrajectory().fresh()
//                //depo3
            .setTangent(Math.toRadians(272))
            .splineToLinearHeading(new Pose2d(-62, depositY, Math.toRadians(90)), Math.toRadians(272))
            .waitSeconds(depositWait);



//        TrajectoryActionBuilder intake4 = deposit3.endTrajectory().fresh()
//                // sub intake
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(subSampleX-20, subSampleY, Math.toRadians(0)), Math.toRadians(0))
//
//        TrajectoryActionBuilder deposit4 = intake4.endTrajectory().fresh()
//                // depo 4
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(depositX, depositY, Math.toRadians(40)), Math.toRadians(268))
//
        TrajectoryActionBuilder subDrive = deposit3.endTrajectory().fresh() // CHANGE TO DEPOSIT4 TO DRIVE BACK TO PARK AFTER SUB INTAKE + DEPO
            //ascent zone park
            .setTangent(Math.toRadians(90))
            .splineToLinearHeading(new Pose2d(-48, -11, Math.toRadians(0)), Math.toRadians(0))
            .lineToX(-20);

        robot.initSubsystems();
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        while (gamepad1.circle == false){
            if (gamepad1.dpad_down && !oldDpadDown) {
                subSampleY -= 1;
            }
            if (gamepad1.dpad_up && !oldDpadUp) {
                subSampleY += 1;
            }
            if (gamepad1.dpad_left && !oldDpadLeft) {
                subSampleX -= 1;
            }
            if (gamepad1.dpad_right && !oldDpadRight) {
                subSampleX += 1;
            }
            oldDpadUp = gamepad1.dpad_up;
            oldDpadDown = gamepad1.dpad_down;
            oldDpadLeft = gamepad1.dpad_left;
            oldDpadRight = gamepad1.dpad_right;
        }

        gamepad1.circle = false;
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

                    new ParallelAction(
                        robot.autoBucketIntake(true),
                        intake1.build()
                    ),

                    new SleepAction(1),

                    robot.autoHighBasketAction(),
                    robot.outtakeSample(true),
                    spike2.build(),

                    new ParallelAction(
                            robot.autoBucketIntake(true),
                            intake2.build()
                    ),
                    new SleepAction(1),

                    robot.autoHighBasketAction(),
                    robot.outtakeSample(true),

                    spike3.build(),
                    new ParallelAction(
                        robot.autoBucketIntake(true),
                        intake3.build()
                    ),

                    new ParallelAction(
                            robot.autoHighBasketAction(),
                            deposit3.build()
                    ),
                    robot.outtakeSample(true),

//                    intake4.build(),
//
//                    robot.autoBucketIntake(true),
//                    new SleepAction(1),
//
//                    new InstantAction(() -> robot.claw.setPower(0)),
//                    new ParallelAction(
//                            robot.autoHighBasketAction(),
//                            deposit4.build()
//                    ),
//                    robot.outtakeSample(true),

                    new SleepAction(1),
                    subDrive.build(),
                    robot.sweeper.sweep(),
                    new SleepAction(1),
                    robot.sweeper.sweep()
                ),
                new LoopAction(() -> {
                    for (LynxModule module : allHubs) {
                        module.clearBulkCache();
                    }
                    robot.lift.update();
                }, this::isStopRequested)
                , new WinchTimeAction(robot.climbWinch, 2.2, -1, telemetry)
            )
        );
    }
}