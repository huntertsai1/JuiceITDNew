package org.firstinspires.ftc.teamcode.subsystems.climb;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.util.hardware.ContinuousServo;

public class ClimbWinch {
    public ContinuousServo servo1, servo2;

    public int wraps1 = 0;
    public double lastPosition1;
    public int wraps2 = 0;
    public double lastPosition2;

    public ClimbWinch(ContinuousServo s1, ContinuousServo s2) {
        servo1 = s1;
        servo2 = s2;
        lastPosition1 = servo1.encoder.getPosition();
        lastPosition2 = servo2.encoder.getPosition();
    }

    public void setPower(float p) {
        servo1.setSpeed(p);
        servo2.setSpeed(p);
    }
    public void update(){
        if (servo1.encoder.getPosition() - lastPosition1 < -270) {
            wraps1 += 1;
        }
        else if (servo1.encoder.getPosition() - lastPosition1 > 270) {
            wraps1 -= 1;
        }
        lastPosition1 = servo1.encoder.getPosition();

        if (servo2.encoder.getPosition() - lastPosition2 < -270) {
            wraps2 += 1;
        }
        else if (servo2.encoder.getPosition() - lastPosition2 > 270) {
            wraps2 -= 1;
        }
        lastPosition2 = servo2.encoder.getPosition();
    }


    public void equalServosStart(){
        servo2.encoder.setPosition(servo1.getAngle());
    }
    public float getPosition(){
        return servo1.getAngle();
    }
    public float getPosition1() {
        return (float) servo1.encoder.getPosition() + wraps1*360;
    }
    public float getPosition2() {
        return (float) servo2.encoder.getPosition() + wraps2*360;
    }
}
