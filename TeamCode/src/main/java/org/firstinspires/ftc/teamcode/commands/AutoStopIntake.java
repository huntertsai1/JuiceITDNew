package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;

import java.util.ArrayList;

public class AutoStopIntake implements Action {
    Robot robot;
    SampleColors[] colors;

    public AutoStopIntake(Robot r) {
        robot = r;
        //colors = c;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return robot.bucketAutoStopIntakeUpdate();
    }
}
