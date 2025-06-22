package org.firstinspires.ftc.teamcode.commands.auton;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;

import java.util.ArrayList;

public class StopIntakeTimeout implements Action {
    Robot robot;
    SampleColors[] colors;
    ElapsedTime timeout;
    long TIMEOUT;
    boolean started = false;

    public StopIntakeTimeout(Robot r, long t, SampleColors... c) {
        robot = r;
        colors = c;
        TIMEOUT = t;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!started) {
            timeout = new ElapsedTime();
        }
        if (timeout.time() <= TIMEOUT) {
            return robot.autoStopIntakeUpdate(null, colors);
        } else {
            robot.stopIntake();
            return false;
        }
    }
}
