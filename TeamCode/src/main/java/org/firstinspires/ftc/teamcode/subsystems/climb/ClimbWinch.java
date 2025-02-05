package org.firstinspires.ftc.teamcode.subsystems.climb;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.util.hardware.ContinuousServo;

public class ClimbWinch {
    public ContinuousServo servo1, servo2;

    public int wraps1 = 0;
    public float lastPosition1;
    public int wraps2 = 0;
    public float lastPosition2;

    public ClimbWinch(ContinuousServo s1, ContinuousServo s2) {
        servo1 = s1;
        servo2 = s2;
        lastPosition1 = servo1.getAngle();
        lastPosition2 = servo2.getAngle();
    }

    public void setPower(float p) {
        servo1.setSpeed(p);
        servo2.setSpeed(p);
    }
    public void update(){
        if (servo1.getAngle() - lastPosition1 < -50 && servo1.getSpeed() > 0 ){
            wraps1 += 1;
        }else if (servo1.getAngle() - lastPosition1 > 50 && servo1.getSpeed() < 0 ){
            wraps1 -= 1;
        }
        lastPosition1 = servo1.getAngle();

        if (servo2.getAngle() - lastPosition2 < -50 && servo2.getSpeed() > 0 ){
            wraps2 += 1;
        }else if (servo2.getAngle() - lastPosition2 > 50 && servo2.getSpeed() < 0 ){
            wraps2 -= 1;
        }
        lastPosition2 = servo2.getAngle();
    }


    public void equalServosStart(){
        servo2.encoder.setPosition(servo1.getAngle());
    }
    public float getPosition(){
        return servo1.getAngle();
    }
    public float getPosition1() {
        return servo1.getAngle() + wraps1*360 + (float)servo1.encoder.offset;
    }
    public float getPosition2() {
        return servo2.getAngle() + wraps2*360 + (float)servo2.encoder.offset;
    }
}
