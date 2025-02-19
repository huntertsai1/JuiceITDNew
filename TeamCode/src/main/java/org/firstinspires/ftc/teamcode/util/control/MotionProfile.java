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
    //Telemetry telemetry;
    public MotionProfile(double start, double end, double maxvel, double maxaccel) {
        // Calculate the time it takes to accelerate to max velocity
        acceleration_dt = maxvel / maxaccel;
        distance = end - start;
        this.start = start;
        this.end = end;

        // If its going backwards, make sure to make accel negative
        if (distance < 0) {
            maxaccel = -maxaccel;
        }

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        acceleration_distance = 0.5 * maxaccel * Math.pow(acceleration_dt, 2);

        if (Math.abs(acceleration_distance) > Math.abs(distance/2)) {
            acceleration_dt = Math.sqrt((distance/2)/ (0.5 * maxaccel));
        }

        acceleration_distance = 0.5 * maxaccel * Math.pow(acceleration_dt, 2);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        maxvel = maxaccel * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        deceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        cruise_distance = distance - 2 * acceleration_distance;
        cruise_dt = cruise_distance / maxvel;
        deceleration_time = acceleration_dt + cruise_dt;

        this.maxvel = maxvel;
        this.maxaccel = maxaccel;
    }

    public double get(double time) {
        // Calculate the time it takes to accelerate to max velocity
        acceleration_dt = maxvel / maxaccel;
        distance = end - start;

//        // If its going backwards, make sure to make accel negative
//        if (distance < 0) {
//            maxaccel = -maxaccel;
//        }

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        acceleration_distance = 0.5 * maxaccel * Math.pow(acceleration_dt, 2);

        if (Math.abs(acceleration_distance) > Math.abs(distance/2)) {
            acceleration_dt = Math.sqrt((distance/2) / (0.5 * maxaccel));
            acceleration_distance = distance/2;
            maxvel = maxaccel*acceleration_dt;
        }

        // we decelerate at the same rate as we accelerate
        deceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        cruise_distance = distance - 2 * acceleration_distance;
      //  cruise_distance = 0;
        cruise_dt = cruise_distance / maxvel;
        deceleration_time = acceleration_dt + cruise_dt;


        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
        if (time >= entire_dt) {
            return start + distance;
        }

        // if we're accelerating
        if (time <= acceleration_dt) {
            //telemetry.addData("mode", 0);
            // use the kinematic equation for acceleration
            return start + (0.5 * maxaccel * Math.pow(time, 2));
        }

        // if we're cruising
        else if (time <= deceleration_time) {
            //telemetry.addData("mode", 100);
            acceleration_distance = 0.5 * maxaccel * Math.pow(acceleration_dt, 2);
            current_dt = time - acceleration_dt;

            // use the kinematic equation for constant velocity (i luv mrs moore)
            return start + acceleration_distance + maxvel * current_dt;
        }

        // if we're decelerating
        else {
            //telemetry.addData("mode", 200);
            //telemetry.addData("deceleration_time", deceleration_time);

            acceleration_distance = 0.5 * maxaccel * Math.pow(acceleration_dt, 2);
            cruise_distance = maxvel * cruise_dt;
            //telemetry.addData("deceleration_time after", deceleration_time);
            current_dt = time - deceleration_time;
            // use the kinematic equations to calculate the instantaneous desired position
            return start + acceleration_distance + cruise_distance + (maxvel * current_dt) - (0.5 * maxaccel * Math.pow(current_dt, 2));
        }
    }
}
