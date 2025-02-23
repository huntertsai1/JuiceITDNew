package org.firstinspires.ftc.teamcode.commands.teleop;

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
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.util.CancellableAction;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.util.StateKeeper;

public class TeleAutoCycle extends CancellableAction {
    boolean started = false;
    boolean cancelled = false;

    double veloLim = 60.0;
    double accelUpperLim = 60.0;
    double accelLowerLim = -40.0;
    double waits = 0.2;
    double intakeWait = 0.3;
    public static double depoTargetX = 9;
    PinpointDrive drive;
    Robot robot;
    Action depoPath;
    Action intakePath;
    Action fullPath;
    Pose2d primePos = new Pose2d(20, -47, Math.toRadians(-45));
    public TeleAutoCycle(PinpointDrive drive, Robot robot, Gamepad gamepad) {
        this.drive = drive;
        this.robot = robot;
//        double depoTargetX = StateKeeper.findOpenHighRung();


        if (depoTargetX == -16236) {
            cancelled = true;
            gamepad.runRumbleEffect(new Gamepad.RumbleEffect.Builder()
                    .addStep(0, 1, 150)
                    .addStep(1, 0, 150)
                    .addStep(0, 1, 150)
                    .addStep(1, 0, 150)
                    .addStep(0, 1, 150)
                    .addStep(1, 0, 150)
                    .build());
        } else {

            depoPath = drive.actionBuilder(primePos)
                    .setTangent(Math.toRadians(180))
                    .splineToLinearHeading(new Pose2d(depoTargetX, -29, Math.toRadians(-92)), Math.toRadians(90),
                            new TranslationalVelConstraint(veloLim),
                            new ProfileAccelConstraint(accelLowerLim, accelUpperLim))
                    .build();
            intakePath = drive.actionBuilder(new Pose2d(depoTargetX, -30, Math.toRadians(-92)))
                    .setTangent(Math.toRadians(-90))
                    .splineToLinearHeading(new Pose2d(20, -47, Math.toRadians(-45)), Math.toRadians(0),
                            new TranslationalVelConstraint(veloLim),
                            new ProfileAccelConstraint(accelLowerLim, accelUpperLim))
                    .build();
            fullPath = new SequentialAction(
                    robot.autoTeleIntakeBoom(),
                    new SleepAction(intakeWait),
                    new ParallelAction(
                            robot.highRungAuto(true),
                            depoPath,
                            new InstantAction(()->{depoTargetX -= 1.5;})
                    ),
                    robot.autoSpecimen(true),

                    new ParallelAction(
                            intakePath,
                            robot.autoTeleIntakePrime()
                    )
            );
        }
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
    public void failsafeAbort() {
        cancelled = true;
        robot.teleAutoIntakePrime(true);
    }
}
