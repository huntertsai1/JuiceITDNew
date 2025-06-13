package org.firstinspires.ftc.teamcode.subsystems.lift;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.enums.Levels;
import org.firstinspires.ftc.teamcode.util.hardware.Motor;

import java.util.concurrent.TimeUnit;

@Config
public class Lift {
    private PIDController controller1;

    public static double p = 0.014, i = 0.00, d = 0.0008;
    public static double f = 0.15;

    public double target = 0;
    private int motorPos = 0;
    public double power1;

    public Motor lift1;
    public Motor lift2;
    public Motor lift3;
    public ElapsedTime timer = new ElapsedTime();
    public boolean goingDown = false;
    public boolean spiked = false;

    public Lift(Motor l1, Motor l2, Motor l3) {
        this.lift1 = l1;
        this.lift2 = l2;
        this.lift3 = l3;

        controller1 = new PIDController(p, i , d);

        lift1.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        lift2.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        lift1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift1.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift2.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift3.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lift1.motor.setCurrentAlert(1.4, CurrentUnit.AMPS);

        timer.reset();
    }

    public void update() {

        motorPos = lift1.motor.getCurrentPosition();
        double ff = f;

        double pid1;

        pid1 = controller1.calculate(motorPos, target);
        power1 = (pid1 + ff);

        if (goingDown) {
            if (motorPos < 300) {
                power1 = -0.5;
                if ((lift1.motor.isOverCurrent() && motorPos <= 50 ) || spiked) {
                    spiked = true;
                    power1 = 0;
                    target = 0;
                }
            }
        }

        lift1.motor.setPower(power1);
        lift2.motor.setPower(power1);
        lift3.motor.setPower(power1);
    }

    public void runToPosition(int ticks) {
        spiked = false;
        target = ticks;
        if (ticks == 0) {
            target = 200;
            goingDown = true;
        } else {
            goingDown = false;
        }
    }

    public void runToPreset(Levels level) {
        if (level == Levels.INIT) {
            runToPosition(0);
        } else if (level == Levels.INTAKE) {
            runToPosition(-30);
        } else if (level == Levels.INTERMEDIATE) {
            runToPosition(0);
        } else if (level == Levels.LOCATING_TARGETS) {
            runToPosition(0);
        } else if (level == Levels.LOW_BASKET) {
            runToPosition(800);
        } else if (level == Levels.HIGH_BASKET) {
            runToPosition(1500);
        } else if (level == Levels.LOW_RUNG) {
            runToPosition(0);
        } else if (level == Levels.HIGH_RUNG) {
            runToPosition(578);
        } else if (level == Levels.CLIMB_EXTENDED) {
            runToPosition(0);
        } else if (level == Levels.CLIMB_RETRACTED) {
            runToPosition(0);
        }
    }

    public void setPower(float power) {
        lift1.motor.setPower(power);
        lift2.motor.setPower(power);
        lift3.motor.setPower(power);
    }

    public void setPower(float power1, float power2, float power3) {
        lift1.motor.setPower(power1);
        lift2.motor.setPower(power2);
        lift3.motor.setPower(power3);
    }

    public int getPos() {
        return motorPos;
    }

}