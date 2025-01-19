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
    static final double e = 30;

    public WinchAlign(ClimbWinch cW, Telemetry telemetry) {
        climbWinch = cW;
        this.ticks = ticks;
        this.telemetry = telemetry;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        telemetry.addData("1", climbWinch.servo1.encoder.getPosition());
        telemetry.addData("2", climbWinch.servo2.encoder.getPosition());

        double diff = climbWinch.servo1.encoder.getPosition() - climbWinch.servo2.encoder.getPosition();
        telemetry.addData("diff", diff);
        if (Math.abs(diff) < e || (Math.abs(diff) > 220 && Math.abs(diff-360) < e)){
            climbWinch.servo1.setSpeed(0);
            return false;
        }
        else if ((diff < 0 && !(diff < -180)) || diff>180){
            climbWinch.servo1.setSpeed(0.5f);
        }else if (diff >0 || diff < -180){
            climbWinch.servo1.setSpeed(-0.5f);
        }
        return true;
    }
}
