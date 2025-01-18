package org.firstinspires.ftc.teamcode.subsystems.climb;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.util.hardware.ContinuousServo;

public class ClimbWinch {
    public ContinuousServo servo1, servo2;
    public double start;

    public ClimbWinch(ContinuousServo s1, ContinuousServo s2) {
        servo1 = s1;
        servo2 = s2;
        start = servo1.getAngle()/3.3*360;
    }

    public void setPower(float p) {
        servo1.setSpeed(p);
        servo2.setSpeed(p);
    }

    public void equalServosStart(){
        servo2.encoder.setPosition(servo1.getAngle());
    }
    public float getPosition() {
        //return (float)((servo1.encoder.getVoltage()+servo2.encoder.getVoltage())/6.6 * 360);
        return (float)(servo1.getAngle()/3.3 * 360);
    }
}
