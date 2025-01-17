package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.climb.ClimbWinch;

public class WinchStopAction implements Action {
    ClimbWinch climbWinch;
    float ticks;
    static int wraps = 0;
    float lastPosition = 0;
    boolean firstRun = true;
    boolean forward;
    static double pos;

    public WinchStopAction(ClimbWinch cW, float ticks) {
        climbWinch = cW;
        this.ticks = ticks;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        pos = (climbWinch.getPosition() + (wraps*360));
        if (Math.abs(pos - ticks) < 50) {
            climbWinch.setPower(0);
            return false;
        }else if (pos < ticks){
            climbWinch.setPower(1);
        }
        else if (pos > ticks){
            climbWinch.setPower(-1);
        }

        if (climbWinch.getPosition() - lastPosition < -270) {
            wraps += 1;
        }
        else if (climbWinch.getPosition() - lastPosition > 270) {
            wraps -= 1;
        }

        lastPosition = climbWinch.getPosition();

        return true;
    }
}
