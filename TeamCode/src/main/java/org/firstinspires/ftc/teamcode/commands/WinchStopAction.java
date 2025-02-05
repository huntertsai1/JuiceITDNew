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
    Telemetry telemetry;

    public WinchStopAction(ClimbWinch cW, float ticks, Telemetry telemetry) {
        climbWinch = cW;
        this.ticks = ticks;
        this.telemetry = telemetry;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        pos1 = climbWinch.getPosition1();
        pos2 = climbWinch.getPosition2();
        telemetry.addData("pos1", pos1);
        telemetry.addData("pos2", pos2);

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

        return true;
    }
}
