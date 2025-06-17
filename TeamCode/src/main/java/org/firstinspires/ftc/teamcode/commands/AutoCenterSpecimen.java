package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;

import java.util.ArrayList;

public class AutoCenterSpecimen implements Action {
    Robot robot;
    SampleColors[] colors;
    int status = 0;

    public AutoCenterSpecimen(Robot r) {
        robot = r;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (robot.claw.detectSampleTail()) {
            robot.claw.setPower(-0.1F);
            return true;
        } else {
            robot.claw.setStall(true);
            return false;
        }
    }
}
