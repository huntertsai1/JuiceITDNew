package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;

import java.util.ArrayList;

public class StopIntake implements Action {
    Robot robot;
    SampleColors[] colors;
    Gamepad gamepad;

    public StopIntake(Robot r, Gamepad g, SampleColors... c) {
        robot = r;
        colors = c;
        gamepad = g;
    }

    public StopIntake(Robot r, SampleColors... c) {
        robot = r;
        colors = c;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return robot.autoStopIntakeUpdate(gamepad, colors);
    }
}
