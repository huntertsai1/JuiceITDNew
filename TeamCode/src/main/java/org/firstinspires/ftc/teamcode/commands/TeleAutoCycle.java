package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.util.CancellableAction;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

public class TeleAutoCycle extends CancellableAction {
    boolean started = false;
    boolean cancelled = false;

    double veloLim = 60.0;
    double accelUpperLim = 60.0;
    double accelLowerLim = -40.0;
    double waits = 0.2;
    double intakeWait = 0.3;

    PinpointDrive drive;
    Robot robot;
    Action depoPath;
    Action intakePath;
    Action fullPath;
    Pose2d relocPos = new Pose2d(19, -48, Math.toRadians(-45));
    public TeleAutoCycle(PinpointDrive drive, Robot robot) {
        this.drive = drive;
        this.robot = robot;
        depoPath = drive.actionBuilder(relocPos)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(0, -30, Math.toRadians(-92)), Math.toRadians(90),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim))
                .build();
        intakePath = drive.actionBuilder(new Pose2d(0, -30, Math.toRadians(-92)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(19, -48, Math.toRadians(-45)), Math.toRadians(0),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim))
                .build();
        fullPath = new SequentialAction(
                robot.teleAutoIntake(),
                new SleepAction(intakeWait),
                robot.highRung(true),
                new SleepAction(waits),

                depoPath,
                robot.autoSpecimen(true),

                new ParallelAction(
                        intakePath,
                        robot.teleAutoIntakePrime(true)
                )
        );
    }


    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (cancelled) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
            return false;
        }

        return fullPath.run(telemetryPacket);
    }

    public void abort() {
        cancelled = true;
    }
}
