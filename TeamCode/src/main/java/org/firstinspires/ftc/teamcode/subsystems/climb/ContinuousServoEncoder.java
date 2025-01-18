package org.firstinspires.ftc.teamcode.subsystems.climb;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class ContinuousServoEncoder {
    AnalogInput encoder;
    boolean reversed;
    double position;
    double offset = 0;

    public ContinuousServoEncoder(AnalogInput e, boolean reversed){
        encoder = e;
        this.reversed = reversed;
    }
    public double getPosition(){
        if (reversed){
            return (1-encoder.getVoltage()/3.3)*360 - offset;
        }
        return encoder.getVoltage()/3.3 *360 - offset;
    }
    public void setPosition(double pos){
        offset = getPosition() - pos;
    }
}
