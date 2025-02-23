package org.firstinspires.ftc.teamcode.subsystems.sweeper;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.configuration.BuiltInConfigurationTypeJsonAdapter;

import org.firstinspires.ftc.teamcode.util.hardware.StepperServo;

public class Sweeper {
    StepperServo servo;

    public boolean swept = false;

    public float RETRACTED_POS = 100;
    public float FULL_EXTENDED_POS = 210;
    public double SWEEP_DELAY = 0.3;

    public Sweeper(StepperServo s) {
        servo = s;
    }

    public void setPosition(float pos) {
        servo.setAngle(pos);
    }

    public Action sweep() {
        swept = true;
        return new SequentialAction(
                new InstantAction(this::extend),
                new SleepAction(SWEEP_DELAY),
                new InstantAction(this::retract)
        );
    }

    public void retract() {
        setPosition(RETRACTED_POS);
    }

    public void extend() {
        setPosition(FULL_EXTENDED_POS);
    }
}
