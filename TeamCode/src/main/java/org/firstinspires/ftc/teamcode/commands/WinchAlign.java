package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.climb.ClimbWinch;
import org.firstinspires.ftc.teamcode.subsystems.climb.ContinuousServoEncoder;

public class WinchAlign implements Action {
    ClimbWinch climbWinch;
    float ticks;
    Telemetry telemetry;

    public WinchAlign(ClimbWinch cW, Telemetry telemetry) {
        climbWinch = cW;
        this.ticks = ticks;
        this.telemetry = telemetry;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        double diff = climbWinch.servo1.encoder.getPosition() + climbWinch.servo2.encoder.getPosition();
        if (Math.abs(diff) < 20 || (Math.abs(diff) > 330 && Math.abs(diff) < 380)){
            return false;
        }
        else if (diff < 0){
            climbWinch.servo1.setSpeed(1);
        }else{
            climbWinch.servo1.setSpeed(-1);
        }
        return true;
    }
}
