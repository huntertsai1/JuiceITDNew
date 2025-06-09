package org.firstinspires.ftc.teamcode.subsystems.lift;

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

public class Lift {
    private PIDController controller1;
    private PIDController controller2;

    public double p1 = 0.00, i1 = 0.00, d1 = 0.00;
    public double f1 = 0.00;

    public double p2 = 0.00, i2 = 0.00, d2 = 0.00;
    public double f2 = 0.00;
    double voltageCompensation;

    public double target = 0;
    public double power1;

    public Motor lift1;
    public Motor lift2;
    public Motor lift3;
    public VoltageSensor voltageSensor;
    public double MAX_VEL = 1500;
    public double MAX_ACCEL = 3000;
    public ElapsedTime timer = new ElapsedTime();
    public boolean goingDown = false;
    public boolean spiked = false;

    public Lift(Motor l1, Motor l2, Motor l3, VoltageSensor voltageSensor) {
        this.lift1 = l1;
        this.lift2 = l2;
        this.lift3 = l3;
        this.voltageSensor = voltageSensor;

        controller1 = new PIDController(p1, i1 , d1);
        controller2 = new PIDController(p2, i2 , d2);

        lift1.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        lift2.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        lift1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift1.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift2.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift3.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        timer.reset();
    }


    public void update() {

        int motorPos = lift1.motor.getCurrentPosition();
        double ff1 = f1;
        double ff2 = f2;

        double pid1;
        double pid2;

        voltageCompensation = 13.3 / voltageSensor.getVoltage();

        if (goingDown) {
            pid2 = controller2.calculate(motorPos, target);
            power1 = (pid2 + ff2) * voltageCompensation;
        }

        else {
            pid1 = controller1.calculate(motorPos, target);
            power1 = (pid1 + ff1) * voltageCompensation;
        }

        lift1.motor.setPower(power1);
        lift2.motor.setPower(power1);
        lift3.motor.setPower(power1);
    }

    public void runToPosition(int ticks) {
        target = ticks;
        if (ticks <= 200) {
            goingDown = true;
        } else {
            goingDown = false;
        }
    }

    public void runToPreset(Levels level) {
        if (level == Levels.INIT) {
            runToPosition(0);
        } else if (level == Levels.INTAKE) {
            runToPosition(0);
        } else if (level == Levels.INTERMEDIATE) {
            runToPosition(0);
        } else if (level == Levels.LOCATING_TARGETS) {
            runToPosition(0);
        } else if (level == Levels.LOW_BASKET) {
            runToPosition(800);
        } else if (level == Levels.HIGH_BASKET) {
            runToPosition(1520);
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
        return lift1.motor.getCurrentPosition();
    }

}