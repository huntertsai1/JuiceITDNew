package org.firstinspires.ftc.teamcode.util.control;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotionProfile {

    double maxvel;
    double maxaccel;

    double acceleration_dt;
    double distance;
    double acceleration_distance;
    double deceleration_dt;
    double cruise_distance;
    double cruise_dt;
    double deceleration_time;
    double start;
    double end;
    double current_dt;

    public MotionProfile(double start, double end, double maxvel, double maxaccel) {
        this.start = start;
        this.end = end;
        distance = end - start;

        // figoure out direction of motion
        double direction = (distance < 0) ? -1 : 1;
        maxaccel *= direction;
        maxvel *= direction;

        // calc the accel time
        acceleration_dt = Math.abs(maxvel / maxaccel);
        acceleration_distance = 0.5 * maxaccel * Math.pow(acceleration_dt, 2);

        // negative fail safe
        if (Math.abs(acceleration_distance) > Math.abs(distance / 2)) {
            double sqrtValue = (Math.abs(distance / 2)) / (0.5 * Math.abs(maxaccel));
            acceleration_dt = (sqrtValue >= 0) ? Math.sqrt(sqrtValue) : 0;
        }

        // recalc values
        acceleration_distance = 0.5 * maxaccel * Math.pow(acceleration_dt, 2);
        maxvel = maxaccel * acceleration_dt;
        deceleration_dt = acceleration_dt;

        // calc cruise params
        cruise_distance = distance - 2 * acceleration_distance;
        cruise_dt = (maxvel != 0) ? (cruise_distance / maxvel) : 0;
        deceleration_time = acceleration_dt + cruise_dt;
    }

    public double get(double time) {
        double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
        if (time >= entire_dt) {
            return start + distance;
        }

        // Acceleration phase
        if (time <= acceleration_dt) {
            return start + (0.5 * maxaccel * Math.pow(time, 2));
        }

        // Cruise phase
        else if (time <= deceleration_time) {
            acceleration_distance = 0.5 * maxaccel * Math.pow(acceleration_dt, 2);
            current_dt = time - acceleration_dt;
            return start + acceleration_distance + maxvel * current_dt;
        }

        // Deceleration phase
        else {
            acceleration_distance = 0.5 * maxaccel * Math.pow(acceleration_dt, 2);
            cruise_distance = maxvel * cruise_dt;
            current_dt = time - deceleration_time;
            return start + acceleration_distance + cruise_distance + (maxvel * current_dt) - (0.5 * maxaccel * Math.pow(current_dt, 2));
        }
    }
}