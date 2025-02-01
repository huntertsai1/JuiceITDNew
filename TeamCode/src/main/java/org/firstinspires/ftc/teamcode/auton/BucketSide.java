package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

@Disabled
@Autonomous(name = "BucketSide", group = "Autonomous")
public class BucketSide extends LinearOpMode {
    Robot robot;
    PinpointDrive drive;
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-30, -60, Math.toRadians(0));
        robot = new Robot(hardwareMap, true);
        drive = new PinpointDrive(hardwareMap, startPose);

        TrajectoryActionBuilder preload = drive.actionBuilder(startPose)
                //preload
                .setTangent(Math.toRadians(165))
                .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.toRadians(200))
                .waitSeconds(3);

        TrajectoryActionBuilder spike1 = preload.endTrajectory().fresh()
                //spike1
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(90)), Math.toRadians(45))
                .waitSeconds(1);

        TrajectoryActionBuilder depo1 = spike1.endTrajectory().fresh()
                //depo1
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.toRadians(225))
                .waitSeconds(3);

        TrajectoryActionBuilder spike2 = depo1.endTrajectory().fresh()
                //spike2
                .setTangent(Math.toRadians(92))
                .splineToLinearHeading(new Pose2d(-58, -48, Math.toRadians(90)), Math.toRadians(100))
                .waitSeconds(1);

        TrajectoryActionBuilder depo2 = spike2.endTrajectory().fresh()
                //depo2
                .setTangent(Math.toRadians(272))
                .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.toRadians(272))
                .waitSeconds(3);

        TrajectoryActionBuilder spike3 = depo2.endTrajectory().fresh()
                //spike3
                .setTangent(Math.toRadians(88))
                .splineToLinearHeading(new Pose2d(-53, -44.5, Math.toRadians(127)), Math.toRadians(95))
                .waitSeconds(1);

        TrajectoryActionBuilder depo3 = spike3.endTrajectory().fresh()
                //depo3
                .setTangent(Math.toRadians(268))
                .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.toRadians(268))
                .waitSeconds(3);

        TrajectoryActionBuilder park = depo3.endTrajectory().fresh()
                //ascent zone park
                .splineToLinearHeading(new Pose2d(-25, -10, Math.toRadians(0)), Math.toRadians(0));

        telemetry.addData("is","starting");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        preload.build(),
                        spike1.build(),
                        depo1.build(),
                        spike2.build(),
                        depo2.build(),
                        spike3.build(),
                        depo3.build()
//                        park
                ));
    }
}