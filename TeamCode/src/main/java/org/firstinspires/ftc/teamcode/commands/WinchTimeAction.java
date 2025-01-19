package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.climb.ClimbWinch;

public class WinchTimeAction implements Action {
    ClimbWinch climbWinch;
    double time;
    Telemetry telemetry;
    float speed;
    ElapsedTime t;
    public WinchTimeAction(ClimbWinch cW, double time, float speed, Telemetry telemetry) {
        climbWinch = cW;
        this.time = time;
        this.speed = speed;
        this.telemetry = telemetry;
        t = new ElapsedTime();
        t.reset();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (t.time() < time){
            climbWinch.setPower(speed);
            return true;
        }
        return false;
    }
}
