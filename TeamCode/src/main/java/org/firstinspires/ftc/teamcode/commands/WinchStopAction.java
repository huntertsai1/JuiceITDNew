package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.climb.ClimbWinch;
import org.firstinspires.ftc.teamcode.subsystems.climb.ContinuousServoEncoder;

public class WinchStopAction implements Action {
    ClimbWinch climbWinch;
    float ticks;
    static double pos1;
    static double pos2;
    static int wraps1 = 0;
    static int wraps2 = 0;
    static double lastPosition1;
    static double lastPosition2;
    Telemetry telemetry;

    public WinchStopAction(ClimbWinch cW, float ticks, Telemetry telemetry) {
        climbWinch = cW;
        this.ticks = ticks;
        this.telemetry = telemetry;
        lastPosition1 = climbWinch.servo1.getAngle();
        lastPosition2 = climbWinch.servo2.getAngle();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        pos1 = climbWinch.servo1.getAngle() + (wraps1 *360);
        pos2 = climbWinch.servo2.getAngle()+ (wraps2 *360);
        telemetry.addData("posW", pos1);
        telemetry.addData("wraps", wraps1);

        if (Math.abs(pos1 - ticks) < 50) {
            climbWinch.servo1.setSpeed(0);
            return false;
        }else if (pos1 < ticks){
            climbWinch.servo1.setSpeed(1);
        }
        else if (pos1 > ticks){
            climbWinch.servo1.setSpeed(-1);
        }

        if (Math.abs(pos2 - ticks) < 50) {
            climbWinch.servo2.setSpeed(0);
            return false;
        }else if (pos2 < ticks){
            climbWinch.servo2.setSpeed(1);
        }
        else if (pos2 > ticks){
            climbWinch.servo2.setSpeed(-1);
        }

        if (pos1 - lastPosition1 < -330) {
            wraps1 += 1;
        }
        else if (pos1 - lastPosition1 > 330) {
            wraps1 -= 1;
        }
        lastPosition1 = pos1;

        if (pos2 - lastPosition2 < -330) {
            wraps2 += 1;
        }
        else if (pos2 - lastPosition2 > 330) {
            wraps2 -= 1;
        }
        lastPosition2 = pos2;

        return true;
    }
}
