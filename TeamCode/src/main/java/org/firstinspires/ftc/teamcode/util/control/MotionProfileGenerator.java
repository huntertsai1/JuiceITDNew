package org.firstinspires.ftc.teamcode.util.control;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotionProfileGenerator {
    public static MotionProfile generateSimpleMotionProfile(double start, double end, double maxvel, double maxaccel) {
        return new MotionProfile(start, end, maxvel, maxaccel);
    }
}
