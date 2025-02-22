package org.firstinspires.ftc.teamcode.subsystems.arm;

import org.firstinspires.ftc.teamcode.util.enums.Levels;
import org.firstinspires.ftc.teamcode.util.hardware.StepperServo;

public class Arm {
    StepperServo servo;
    StepperServo wrist;

    float target = 0;
    float wristDown = 173;
    float wristUp = 0;

    public Arm(StepperServo s1, StepperServo s2) {
        servo = s1;
        wrist = s2;
    }

    public void runToPosition(float t) {
        servo.setAngle(t);

        target = t;
    }

    public void runToPreset(Levels level) {
        if (level == Levels.INIT) {
            runToPosition(280);
            setWristAngle(wristDown);
        } else if (level == Levels.INTAKE) {
            runToPosition(135);
            setWristAngle(wristDown);
        } else if (level == Levels.INTAKE_INTERMEDIATE) {
            runToPosition(185);
            setWristAngle(wristDown);
        } else if (level == Levels.INTERMEDIATE) {
            runToPosition(215);
            setWristAngle(wristDown);
        } else if (level == Levels.LOW_BASKET) {
            runToPosition(322);
            setWristAngle(wristUp);
        } else if (level == Levels.HIGH_BASKET) {
            runToPosition(322);
            setWristAngle(wristUp);
        } else if (level == Levels.HIGH_RUNG) {
            runToPosition(335);
            setWristAngle(55);
        }
    }

    public float getPosition() {
        return target;
    }

    public void setWristAngle(float angle) {
        wrist.setAngle(angle);
    }

    public void runToWristPreset(Levels level) {
        if (level == Levels.INTAKE_INTERMEDIATE) {
            setWristAngle(10);
        }
    }
}
