package org.firstinspires.ftc.teamcode.subsystems.extension;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.enums.Levels;
import org.firstinspires.ftc.teamcode.util.hardware.StepperServo;

public class Extension {
    StepperServo servo1;
    StepperServo servo2;
    public float low = 60;

    float target = 0;

    public int high = 285;
    public boolean nextShort = false;
    public Extension(StepperServo s1, StepperServo s2) {
        servo1 = s1;
        servo2 = s2;

        servo1.servo.setDirection(Servo.Direction.REVERSE);
        servo2.servo.setDirection(Servo.Direction.REVERSE);
    }

    public void runToPosition(float t) {
        servo1.setAngle(t);
        servo2.setAngle(t);

        target = t;
    }

    public void runToPreset(Levels level) {
        if (level == Levels.INIT) {
            runToPosition(low);
        } else if (level == Levels.INTAKE) {
            if (nextShort){
                runToPosition(200);
                nextShort = false;
            }
            else{
                runToPosition(high);
            }
        } else if (level == Levels.INTERMEDIATE) {
            runToPosition(low);
        } else if (level == Levels.LOW_BASKET) {
            runToPosition(low);
        } else if (level == Levels.HIGH_BASKET) {
            runToPosition(low);
        } else if (level == Levels.HIGH_RUNG) {
            runToPosition(62);
        }
    }

    public float getPosition() {
        return target;
    }
    public float getPosition(boolean useEncoder) {
        return servo1.getAngle(true);
    }
}
