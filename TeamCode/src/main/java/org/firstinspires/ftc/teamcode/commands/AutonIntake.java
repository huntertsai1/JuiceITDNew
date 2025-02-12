package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.util.CancellableAction;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.util.StateKeeper;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;

public class AutonIntake extends CancellableAction {
    boolean cancelled = false;

    PinpointDrive drive;
    Robot robot;
    SampleColors target = SampleColors.YELLOW;
    TrajectoryActionBuilder subDrive;
    TrajectoryActionBuilder subSearching;
    Action fullPath;

    boolean ejecting;
    ElapsedTime ejectTimer = new ElapsedTime();
    public AutonIntake(PinpointDrive drive, Robot robot, double xTarget, double headingBias, TrajectoryActionBuilder lastTraj, SampleColors target) {
        this.drive = drive;
        this.robot = robot;
        this.target = target;

        subDrive = lastTraj.endTrajectory().fresh()
                //ascent zone park
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(xTarget, -7, Math.toRadians(0 + headingBias)), Math.toRadians(0 + headingBias));
        subSearching = subDrive.endTrajectory().fresh()
                .waitSeconds(1)
                .turn(Math.toRadians(-5))
                .waitSeconds(1)
                .turn(Math.toRadians(10));
        fullPath = new ParallelAction(
                new InstantAction(() -> {
                    if (ejecting && ejectTimer.time() < 0.25) {
                        robot.claw.setPower(-1);
                    } else if (ejecting) {
                        robot.claw.setPower(1);
                        ejecting = false;
                    }
                }),
                new SequentialAction(
                subDrive.build(),
                new ParallelAction(
                        robot.autoBucketIntake(true),
                        subSearching.build()
                )
            )
        );
    }


    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (cancelled) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
            return false;
        }

        int sensor = robot.claw.smartStopDetect(SampleColors.YELLOW, target);
        if (sensor == 1) {
            failsafeAbort();
        } else if (sensor == -1) {
            ejecting = true;
            ejectTimer.reset();
        }

        return fullPath.run(telemetryPacket);
    }
    public void failsafeAbort() {
        cancelled = true;
        robot.stopIntake();
    }

    public TrajectoryActionBuilder endTrajectory() {
        return subDrive.endTrajectory();
    }
}
