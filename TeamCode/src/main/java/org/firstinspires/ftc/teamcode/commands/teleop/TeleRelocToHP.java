package org.firstinspires.ftc.teamcode.commands.teleop;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.util.CancellableAction;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

public class TeleRelocToHP extends CancellableAction {
    boolean started = false;
    boolean cancelled = false;

    double veloLim = 60.0;
    double accelUpperLim = 60.0;
    double accelLowerLim = -40.0;

    PinpointDrive drive;
    Robot robot;
    Action path;
    Action fullPath;
    Pose2d relocPos = new Pose2d(6.375,-31.375, Math.toRadians(-92));
    public TeleRelocToHP(PinpointDrive drive, Robot robot) {
        this.drive = drive;
        this.robot = robot;
        path = drive.actionBuilder(relocPos)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(21, -46, Math.toRadians(-45)), Math.toRadians(0),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim))
                .build();
        fullPath = new ParallelAction(
                path,
                robot.teleAutoIntakePrime(true)
        );
    }


    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!started) {
            drive.pose = relocPos;
            drive.updatePoseEstimate();
            started = true;
        }

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
