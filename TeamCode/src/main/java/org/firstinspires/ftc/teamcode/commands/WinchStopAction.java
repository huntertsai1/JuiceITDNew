package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.climb.ClimbWinch;

public class WinchStopAction implements Action {
    ClimbWinch climbWinch;
    float ticks;
    int wraps = 0;
    float lastPosition = 0;
    boolean firstRun = true;

    public WinchStopAction(ClimbWinch cW, float ticks) {
        climbWinch = cW;
        this.ticks = ticks;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (firstRun) {
            climbWinch.setPower(1);
            firstRun = false;
        }

        if (Math.abs((climbWinch.getPosition() + (wraps*360)) - ticks) < 1) {
            climbWinch.setPower(0);
            return false;
        }

        if (climbWinch.getPosition() < lastPosition) {
            wraps += 1;
        }

        lastPosition = climbWinch.getPosition();

        return true;
    }
}
