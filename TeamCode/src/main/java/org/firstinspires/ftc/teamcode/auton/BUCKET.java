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
import org.firstinspires.ftc.teamcode.commands.auton.AutonIntake;
import org.firstinspires.ftc.teamcode.commands.util.LoopAction;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;

@Disabled
@Autonomous(name = "BUCKET", group = "Autonomous")
public class BUCKET extends LinearOpMode {
    Robot robot;
    PinpointDrive drive;
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-38, -62, Math.toRadians(0));
        robot = new Robot(hardwareMap, true);
        drive = new PinpointDrive(hardwareMap, startPose);

        double intakeWait = 0.3;

        double yTargetCycle1 = -8;
        double yTargetCycle2 = -8;
        double yTargetCycle3 = -8;
        double yTargetCycle4 = -8;
        double headingAdjustmentCycle1 = 0;
        double headingAdjustmentCycle2 = 0;
        double headingAdjustmentCycle3 = 0;
        double headingAdjustmentCycle4 = 0;
        boolean adventureUpdated = false;

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

//        TrajectoryActionBuilder subDrive = depo3.endTrajectory().fresh()
//                //ascent zone park
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(-31, -7, Math.toRadians(0)), Math.toRadians(0));
        AutonIntake subDrive1 = new AutonIntake(drive, robot, yTargetCycle1, headingAdjustmentCycle1, depo3, SampleColors.RED);

        TrajectoryActionBuilder depo4 = subDrive1.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-51, -51, Math.toRadians(45)), Math.toRadians(268))
                .waitSeconds(1.2);

        AutonIntake subDrive2;

        TrajectoryActionBuilder depo5;

        AutonIntake subDrive3;

        TrajectoryActionBuilder depo6;

        AutonIntake subDrive4;

        TrajectoryActionBuilder depo7;

        telemetry.addData("is","starting");
        telemetry.update();

        robot.initSubsystems();

        int editingCycle = 1;

        while (!isStarted() && !isStopRequested()) {
            // CHOOSE YOUR ADVENTURE!
            if (gamepad1.triangle) {
                editingCycle = 1;
            } else if (gamepad1.circle) {
                editingCycle = 2;
            } else if (gamepad1.cross) {
                editingCycle = 3;
            } else if (gamepad1.square) {
                editingCycle = 4;
            }

            if (gamepad1.dpad_up && editingCycle == 1) {
                yTargetCycle1 = -10;
                adventureUpdated = false;
            }
            if (gamepad1.dpad_down && editingCycle == 1) {
                yTargetCycle1 = -8;
                adventureUpdated = false;
            }
            if (gamepad1.dpad_left && editingCycle == 1) {
                yTargetCycle1 = -7;
                adventureUpdated = false;
            }
            if (gamepad1.dpad_right && editingCycle == 1) {
                yTargetCycle1 = -9;
                adventureUpdated = false;
            }

            if (gamepad1.dpad_up && editingCycle == 2) {
                yTargetCycle2 = -10;
                adventureUpdated = false;
            }
            if (gamepad1.dpad_down && editingCycle == 2) {
                yTargetCycle2 = -8;
                adventureUpdated = false;
            }
            if (gamepad1.dpad_left && editingCycle == 2) {
                yTargetCycle2 = -7;
                adventureUpdated = false;
            }
            if (gamepad1.dpad_right && editingCycle == 2) {
                yTargetCycle2 = -9;
                adventureUpdated = false;
            }

            if (gamepad1.dpad_up && editingCycle == 3) {
                yTargetCycle3 = -10;
                adventureUpdated = false;
            }
            if (gamepad1.dpad_down && editingCycle == 3) {
                yTargetCycle3 = -8;
                adventureUpdated = false;
            }
            if (gamepad1.dpad_left && editingCycle == 3) {
                yTargetCycle3 = -7;
                adventureUpdated = false;
            }
            if (gamepad1.dpad_right && editingCycle == 3) {
                yTargetCycle3 = -9;
                adventureUpdated = false;
            }

            if (gamepad1.dpad_up && editingCycle == 4) {
                yTargetCycle4 = -10;
                adventureUpdated = false;
            }
            if (gamepad1.dpad_down && editingCycle == 4) {
                yTargetCycle4 = -8;
                adventureUpdated = false;
            }
            if (gamepad1.dpad_left && editingCycle == 4) {
                yTargetCycle4 = -7;
                adventureUpdated = false;
            }
            if (gamepad1.dpad_right && editingCycle == 4) {
                yTargetCycle4 = -9;
                adventureUpdated = false;
            }

            if (gamepad1.right_trigger > 0.2 && editingCycle == 1) {
                headingAdjustmentCycle1 += gamepad1.right_trigger;
                adventureUpdated = false;
            } else if (gamepad1.left_trigger > 0.2 && editingCycle == 1) {
                headingAdjustmentCycle1 -= gamepad1.left_trigger;
                adventureUpdated = false;
            }

            if (gamepad1.right_trigger > 0.2 && editingCycle == 2) {
                headingAdjustmentCycle2 += gamepad1.right_trigger;
                adventureUpdated = false;
            } else if (gamepad1.left_trigger > 0.2 && editingCycle == 2) {
                headingAdjustmentCycle2 -= gamepad1.left_trigger;
                adventureUpdated = false;
            }

            if (gamepad1.right_trigger > 0.2 && editingCycle == 3) {
                headingAdjustmentCycle3 += gamepad1.right_trigger;
                adventureUpdated = false;
            } else if (gamepad1.left_trigger > 0.2 && editingCycle == 3) {
                headingAdjustmentCycle3 -= gamepad1.left_trigger;
                adventureUpdated = false;
            }

            if (gamepad1.right_trigger > 0.2 && editingCycle == 4) {
                headingAdjustmentCycle4 += gamepad1.right_trigger;
                adventureUpdated = false;
            } else if (gamepad1.left_trigger > 0.2 && editingCycle == 4) {
                headingAdjustmentCycle4 -= gamepad1.left_trigger;
                adventureUpdated = false;
            }

            if (gamepad1.right_bumper) {
                subDrive1 = new AutonIntake(drive, robot, yTargetCycle1, headingAdjustmentCycle1, depo3, SampleColors.RED);
                depo4 = subDrive1.endTrajectory().fresh()
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-51, -51, Math.toRadians(45)), Math.toRadians(268))
                        .waitSeconds(1.2);

                subDrive2 = new AutonIntake(drive, robot, yTargetCycle2, headingAdjustmentCycle2, depo4, SampleColors.RED);
                depo5 = subDrive2.endTrajectory().fresh()
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-51, -51, Math.toRadians(45)), Math.toRadians(268))
                        .waitSeconds(1.2);

                subDrive3 = new AutonIntake(drive, robot, yTargetCycle2, headingAdjustmentCycle2, depo5, SampleColors.RED);
                depo6 = subDrive3.endTrajectory().fresh()
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-51, -51, Math.toRadians(45)), Math.toRadians(268))
                        .waitSeconds(1.2);

                subDrive4 = new AutonIntake(drive, robot, yTargetCycle2, headingAdjustmentCycle2, depo6, SampleColors.RED);
                depo7 = subDrive4.endTrajectory().fresh()
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-51, -51, Math.toRadians(45)), Math.toRadians(268))
                        .waitSeconds(1.2);
                adventureUpdated = true;
            }

            telemetry.addData("CURRENTLY EDITING", editingCycle);
            if (editingCycle == 1) {
                telemetry.addData("Y TARGET", yTargetCycle1);
                telemetry.addData("HEADING BIAS", headingAdjustmentCycle1);
            } else if (editingCycle == 2) {
                telemetry.addData("Y TARGET", yTargetCycle2);
                telemetry.addData("HEADING BIAS", headingAdjustmentCycle2);
            } else if (editingCycle == 3) {
                telemetry.addData("Y TARGET", yTargetCycle3);
                telemetry.addData("HEADING BIAS", headingAdjustmentCycle3);
            } else {
                telemetry.addData("Y TARGET", yTargetCycle4);
                telemetry.addData("HEADING BIAS", headingAdjustmentCycle4);
            }
            telemetry.addData("SAVED", adventureUpdated);
            telemetry.update();

        }

        waitForStart();

        if (isStopRequested()) return;

        if (!adventureUpdated) {
            subDrive1 = new AutonIntake(drive, robot, yTargetCycle1, headingAdjustmentCycle1, depo3, SampleColors.RED);
            depo4 = subDrive1.endTrajectory().fresh()
                    .setTangent(Math.toRadians(180))
                    .splineToLinearHeading(new Pose2d(-51, -51, Math.toRadians(45)), Math.toRadians(268))
                    .waitSeconds(1.2);

            subDrive2 = new AutonIntake(drive, robot, yTargetCycle2, headingAdjustmentCycle2, depo4, SampleColors.RED);
            depo5 = subDrive2.endTrajectory().fresh()
                    .setTangent(Math.toRadians(180))
                    .splineToLinearHeading(new Pose2d(-51, -51, Math.toRadians(45)), Math.toRadians(268))
                    .waitSeconds(1.2);

            subDrive3 = new AutonIntake(drive, robot, yTargetCycle2, headingAdjustmentCycle2, depo5, SampleColors.RED);
            depo6 = subDrive3.endTrajectory().fresh()
                    .setTangent(Math.toRadians(180))
                    .splineToLinearHeading(new Pose2d(-51, -51, Math.toRadians(45)), Math.toRadians(268))
                    .waitSeconds(1.2);

            subDrive4 = new AutonIntake(drive, robot, yTargetCycle2, headingAdjustmentCycle2, depo6, SampleColors.RED);
            depo7 = subDrive4.endTrajectory().fresh()
                    .setTangent(Math.toRadians(180))
                    .splineToLinearHeading(new Pose2d(-51, -51, Math.toRadians(45)), Math.toRadians(268))
                    .waitSeconds(1.2);
        }

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
                        robot.autoBucketIntake(true),
                        new SleepAction(0.5),

                        new InstantAction(() -> robot.claw.setPower(0)),
                        new ParallelAction(
                            robot.autoHighBasketAction(),
                            depo1.build()
                        ),

                        robot.outtakeSample(true),

                        spike2.build(),
                        robot.autoBucketIntake(true),
                        new SleepAction(0.5),

                        new InstantAction(() -> robot.claw.setPower(0)),
                        new ParallelAction(
                            robot.autoHighBasketAction(),
                            depo2.build()
                        ),

                        robot.outtakeSample(true),

                        spike3.build(),
                        robot.autoBucketIntake(true),
                        new SleepAction(0.5),

                        new InstantAction(() -> robot.claw.setPower(0)),
                        new ParallelAction(
                            robot.autoHighBasketAction(),
                            depo3.build()
                        ),

                        robot.outtakeSample(true),

                        subDrive1,

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