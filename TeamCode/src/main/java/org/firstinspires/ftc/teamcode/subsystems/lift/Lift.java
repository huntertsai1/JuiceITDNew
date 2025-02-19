package org.firstinspires.ftc.teamcode.subsystems.lift;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.util.control.MotionProfile;
import org.firstinspires.ftc.teamcode.util.control.MotionProfileGenerator;
import org.firstinspires.ftc.teamcode.util.enums.Levels;
import org.firstinspires.ftc.teamcode.util.hardware.Motor;

public class Lift {
    private PIDController controller1;

    public double p = 0.023, i = 0.00, d = 0.001;
    public double f = 0.23;
    double voltageCompensation;

    public double target = 0;
    public Levels currentLevel = Levels.ZERO;
    private final double ticks_in_degrees = 700 / 180.0;
    public double power1;

    public Motor lift1;
    public Motor lift2;
    public Motor lift3;
    public VoltageSensor voltageSensor;

    private boolean threadState = false;

    public double MAX_VEL = 1500;
    public double MAX_ACCEL = 3000;
    public MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(0,0, MAX_VEL, MAX_ACCEL);
    public ElapsedTime timer = new ElapsedTime();


    public Lift(Motor l1, Motor l2, Motor l3, VoltageSensor voltageSensor) {
        this.lift1 = l1;
        this.lift2 = l2;
        this.lift3 = l3;
        this.voltageSensor = voltageSensor;

        controller1 = new PIDController(p, i , d);
        lift1.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        lift1.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift3.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        lift3.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift3.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void update() {
//        target = profile.get(timer.time());

        int motorPos = lift1.motor.getCurrentPosition();

        double pid1;
        if (target <= 100) {
            pid1 = controller1.calculate(motorPos, profile.get(timer.time()));
        } else {
            pid1 = controller1.calculate(motorPos, target);
        }
//        double pid2 = controller2.calculate(slides2Pos, target);
//        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
        double ff = f;

        voltageCompensation = 13.5 / voltageSensor.getVoltage();
        power1 = (pid1 + ff) * voltageCompensation;
//        power2 = pid2 + ff;

        if (target == 0){
            lift1.motor.setPower(-power1);
            lift2.motor.setPower(-power1); //was at *0.3 pre push
            lift3.motor.setPower(-power1);
        }
        else {
            lift1.motor.setPower(-power1);
            lift2.motor.setPower(-power1);
            lift3.motor.setPower(-power1);
        }
    }

    public void runToPosition(int ticks) {
        target = ticks;
        if (ticks <= 100) {
            profile = MotionProfileGenerator.generateSimpleMotionProfile(getPos(), ticks, MAX_VEL, MAX_VEL);
            timer.reset();
        }
    }

    public void runToPreset(Levels level) {
        if (level == Levels.INIT) {
            runToPosition(0);
        } else if (level == Levels.INTAKE) {
            runToPosition(-15);
        } else if (level == Levels.INTERMEDIATE) {
            runToPosition(0);
        } else if (level == Levels.LOCATING_TARGETS) {
            runToPosition(0);
        } else if (level == Levels.LOW_BASKET) {
            runToPosition(1300);
        } else if (level == Levels.HIGH_BASKET) {
            runToPosition(2180);
        } else if (level == Levels.LOW_RUNG) {
            runToPosition(0);
        } else if (level == Levels.HIGH_RUNG) {
            runToPosition(960);
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

    public void launchAsThread(Telemetry telemetry) {
        threadState = true;
        telemetry.addData("Slides Threads State:", "STARTING");
        telemetry.update();
        Thread t1 = new Thread(() -> {
            telemetry.addData("Slides Threads State:", "STARTED");
            telemetry.update();
            while (threadState == true) {
                update();
            }
            telemetry.addData("Slides Threads State:", "STOPPED");
            telemetry.update();
        });
        t1.start();
    }

    public void destroyThreads(Telemetry telemetry) {
        telemetry.addData("Slides Threads State:", "STOPPING");
        telemetry.update();
        target = -10;
        threadState = false;
    }

    public void launchAsThreadBasic() {
        threadState = true;
        Thread t1 = new Thread(() -> {
            while (threadState == true) {
                update();
            }
        });
        t1.start();
    }

    public void destroyThreadsBasic() {
        threadState = false;
    }
    public void resetAllEncoders(){
        lift1.resetEncoder();
        lift2.resetEncoder();
    }

    public int getPos() {
        return lift1.motor.getCurrentPosition();
    }

}