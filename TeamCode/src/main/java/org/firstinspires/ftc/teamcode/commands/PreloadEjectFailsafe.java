package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class PreloadEjectFailsafe implements Action {
    Robot robot;

    boolean started = false;
    ElapsedTime timer = new ElapsedTime();

    public PreloadEjectFailsafe(Robot r) {
        robot = r;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//        if (robot.claw.detectSample() != null && !started) {
        if (!started) {
            robot.claw.setPower(-0.5F);
            timer.reset();
            started = true;
        }
//        } else if (!started) {
//            return false;
//        }

        if (timer.time(TimeUnit.MILLISECONDS) >= 500) {
            robot.claw.setPower(0F);
            return false;
        }
        return true;
    }
}
