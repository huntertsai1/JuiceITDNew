package org.firstinspires.ftc.teamcode.commands.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class CancellableAction implements Action {

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return false;
    }

    public void abort() {

    }
}
