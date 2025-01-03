package org.firstinspires.ftc.teamcode.util;

// Angle class for normalization methods
public class Angle {

    // Normalize angle to range [0, 2π)
    public static double norm(double angle) {
        angle = angle % (2 * Math.PI); // Wrap within 2π
        if (angle < 0) {
            angle += 2 * Math.PI;      // Adjust negatives
        }
        return angle;
    }

    // Normalize angular difference to range [-π, π)
    public static double normDelta(double angle) {
        angle = (angle + Math.PI) % (2 * Math.PI); // Wrap within ±π
        if (angle < 0) {
            angle += 2 * Math.PI;                  // Adjust negatives
        }
        return angle - Math.PI;                   // Shift back
    }
}