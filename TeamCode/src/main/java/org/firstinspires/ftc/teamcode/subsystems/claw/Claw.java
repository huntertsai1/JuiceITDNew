package org.firstinspires.ftc.teamcode.subsystems.claw;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.hardware.BrushlandColorSensor;
import org.firstinspires.ftc.teamcode.util.hardware.ContinuousServo;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;

@Config
public class Claw {
    public ContinuousServo servo1;
    public ContinuousServo servo2;
    public BrushlandColorSensor colorSensorHead;
    public BrushlandColorSensor colorSensorTail;

    public static double TAIL_SENSOR_THRESHOLD = 8;
    public static double SLOW_SPEED = 0.17;
    float power = 0;

    ElapsedTime sensorTimeout;

    public Claw(ContinuousServo s1, ContinuousServo s2, BrushlandColorSensor sensorHead, BrushlandColorSensor sensorTail) {
        servo1 = s1;
        servo2 = s2;

        servo1.servo.setDirection(DcMotorSimple.Direction.REVERSE);
        servo2.servo.setDirection(DcMotorSimple.Direction.REVERSE);

        colorSensorHead = sensorHead;
        colorSensorTail = sensorTail;
//        colorSensor.initialize();
//        ((LynxI2cDeviceSynch) sensor.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);

        sensorTimeout = new ElapsedTime();
    }

    public void setPower(float p) {
        servo1.servo.setPower(p);
        servo2.servo.setPower(-p);
        power = p;
    }

    public void startIntake() {
        setPower(1);
    }

    public void stopIntake() {
        setPower(0);
    }

    public void slowIntake() {setPower((float)SLOW_SPEED);}

    public void smartStopIntake(SampleColors... colors) {
        SampleColors s = detectSampleHead();
        if (Arrays.stream(colors).anyMatch(x -> x == s )) {
            stopIntake();
        } else if (s != null) {
            eject();
        }
    }

    /**
     *
     * @param colors colors that you want to stop the intake for
     * @return 0: nothing in intake, 1: intaked targeted color, -1: head target detected (slow)
     */

    public int intakeStatus = 0;
    ElapsedTime primeTimeout = new ElapsedTime();
    public int smartStopDetect(SampleColors... colors) {
        SampleColors s = detectSampleHead();
        boolean isTarget = Arrays.stream(colors).anyMatch(x -> x == s );
        System.out.println("s: " + s + ";   isTarget: " + isTarget + ";  Intake Status: " + + intakeStatus);

        if (isTarget && intakeStatus == 0) {
            primeTimeout.reset();
            intakeStatus = -1;
            return -1;
        } else if (intakeStatus == -1 && detectSampleTail()) {
            intakeStatus = 1;
            return 1;
        } else if (!isTarget && intakeStatus == -1 && primeTimeout.time(TimeUnit.MILLISECONDS) > 750) {
            intakeStatus = 0;
            return 0;
        } else if (intakeStatus == -1) {
            return -1;
        } else if (intakeStatus == 1) {
            return 1;
        } else if (intakeStatus == 16236) {
            return 16236;
        }
        return 0;
    }

    public void eject() {
        setPower((float) -0.25);
        try {
            Thread.sleep(500);
            setPower(0);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void ejectOps() {
        setPower((float) -0.4);
        try {
            Thread.sleep(500);
            setPower(1);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public Action ejectOps(boolean action) {
        return new SequentialAction(
                new InstantAction(() -> setPower((float) -0.4)),
                new SleepAction(0.5),
                new InstantAction(() -> setPower((float) 1))
        );
    }
    public Action ejectOpsAuton(boolean action) {
        return new SequentialAction(
                new InstantAction(() -> setPower((float) -1)),
                new SleepAction(0.5),
                new InstantAction(() -> setPower((float) 1))
        );
    }

    public Action eject(boolean action) {
        return new SequentialAction(
                new InstantAction(() -> setPower((float) -0.4)),
                new SleepAction(0.3),
                new InstantAction(() -> setPower(0))
        );
    }

    /**
     * Toggles a stall state on the claw in order to <b>temporarily</b> increase claw grip.
     * Typically used for clipped deposit.<br/>
     * <h3>WARNING: Don't have the servo in a stalled state for too long in order to extend servo lifetime</h3>
     */
    public void stall() {
        if (power == -0.1) {
            setPower((float) -0.1);
        } else {
            setPower(0);
        }
    }

    /**
     * Toggles a stall state on the claw in order to <b>temporarily</b> increase claw grip.
     * Typically used for clipped deposit.<br/>
     * <h3>WARNING: Don't have the servo in a stalled state for too long in order to extend servo lifetime</h3>
     * @param action input a <code>true</code> in order to activate action return method
     * @return returns <code>InstantAction</code> that toggles the stall
     */
    public Action stall(boolean action) {
        if (power == 0.2) {
            return new InstantAction( () -> setPower((float) 0.2));
        } else {
            return new InstantAction( () -> setPower(0));
        }
    }

    /**
     * Toggles a stall state on the claw in order to <b>temporarily</b> increase claw grip.
     * Typically used for clipped deposit.<br/>
     * <h3>WARNING: Don't have the servo in a stalled state for too long in order to extend servo lifetime</h3.
     * @param state <code>true</code> is a stalled claw and <code>false</code> is a unstalled claw
     */
    public void setStall(boolean state) {
        if (state) {
            setPower((float) 0.03);
        } else {
            setPower(0);
        }
    }

    /**
     * Toggles a stall state on the claw in order to <b>temporarily</b> increase claw grip.
     * Typically used for clipped deposit.<br/>
     * <h3>WARNING: Don't have the servo in a stalled state for too long in order to extend servo lifetime</h3>
     * @param action input a <code>true</code> in order to activate action return method
     * @param state <code>true</code> is a stalled claw and <code>false</code> is a unstalled claw
     * @return returns <code>InstantAction</code> that toggles the stall
     */
    public Action setStall(boolean state, boolean action) {
        if (state) {
            return new InstantAction( () -> setPower((float) 0.2));
        } else {
            return new InstantAction( () -> setPower(0));
        }
    }

    public SampleColors detectSampleHead() {
//        float red = colorSensor.getNormalizedColors().red;
//        float blue = colorSensor.getNormalizedColors().blue;
//        float green = colorSensor.getNormalizedColors().green;
//        double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.MM);
//        if (distance < 20) {
//            if (blue > 0.01 && red < 0.003) {
//                // Extreme blue output -> blue sample
//                return SampleColors.BLUE;
//            } else if (red >= 0.02 && green > 0.02) {
//                return SampleColors.YELLOW;
//            } else if (red >= 0.01 && green < 0.01 && blue < 0.01) {
//                return SampleColors.RED;
//            } else {
//                return SampleColors.UNIDENTIFIABLE;
////            }
//            return SampleColors.YELLOW;
//        } else {
//            return null;
//        }
        boolean p0 = colorSensorHead.getPin0Digital();
        boolean p1 = colorSensorHead.getPin1();

        if (p0 && p1) {
            return SampleColors.YELLOW;
        } else if (p0 && !p1) {
            return SampleColors.RED;
        } else if (!p0 && p1) {
            return SampleColors.BLUE;
        }

        return null;
    }

    public boolean detectSampleTail() {
//        return colorSensorTail.rollingWindowBelow(TAIL_SENSOR_THRESHOLD);
        return colorSensorTail.getPin0Analog() < TAIL_SENSOR_THRESHOLD;
    }
}