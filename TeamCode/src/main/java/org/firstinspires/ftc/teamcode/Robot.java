package org.firstinspires.ftc.teamcode;

// IMPORT SUBSYSTEMS

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.CommandMaster;
import org.firstinspires.ftc.teamcode.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.climb.ClimbWinch;
import org.firstinspires.ftc.teamcode.subsystems.extension.Extension;
import org.firstinspires.ftc.teamcode.subsystems.lift.Lift;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.util.enums.ClimbType;
import org.firstinspires.ftc.teamcode.util.hardware.BrushlandColorSensor;
import org.firstinspires.ftc.teamcode.util.hardware.Component;
import org.firstinspires.ftc.teamcode.util.hardware.ContinuousServo;
import org.firstinspires.ftc.teamcode.util.enums.Levels;
import org.firstinspires.ftc.teamcode.util.hardware.Motor;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;
import org.firstinspires.ftc.teamcode.util.hardware.StepperServo;

import java.util.concurrent.TimeUnit;

public class Robot {

    // SUBSYSTEM DECLARATIONS
    public Component[] components;
    public Lift lift;
    public Extension extension;
    public Arm arm;
    public Claw claw;
    public ClimbWinch climbWinch;

    public CommandMaster commands;
    public HardwareMap hardwareMap;

    // STATE VARS
    boolean auton;
    boolean intaking = false;
    public boolean activateSensor = true;
    public Levels state = Levels.INIT;
    public Gamepiece mode = Gamepiece.SAMPLE;
    public SampleColors targetColor = SampleColors.YELLOW;
    public ClimbType climbMode = ClimbType.LEVEL_3;
    Motor backLeft;
    Motor backRight;
    Motor frontLeft;
    Motor frontRight;

    public Robot(HardwareMap map, boolean auton){
        this.auton = auton;

        this.components = new Component[]{
                new Motor(0, "leftBack", map, true),                //0 left odometer
                new Motor(1, "rightBack", map, false),              //1 right odometer
                new Motor(2, "leftFront", map, true),               //2 middle odometer
                new Motor(3, "rightFront", map, false),             //3

                new Motor(0, "lift1", map, false),                  //4
                new Motor(1, "lift2", map, false),                  //5

                new StepperServo(2, "ext1", map),    //6
                new StepperServo(3, "ext2", map),    //7

                new StepperServo(4, "arm", map),      //8

                new StepperServo(5, "elbow", map),  //9
                new ContinuousServo(1, "claw1", map),                     //10
                new ContinuousServo(5, "claw2", map),                     //11

                new ContinuousServo(0, "climb1", map, "climb1Encoder", false),//12
                new ContinuousServo(1, "climb2", map, "climb2Encoder", false) //13
        };

        VoltageSensor voltageSensor = map.voltageSensor.iterator().next();
//        BrushlandColorSensor colorSensor = new BrushlandColorSensor(0, "color", map);
        RevColorSensorV3 colorSensor = map.get(RevColorSensorV3.class, "colorSensor");
        // INIT SUBSYSTEMS

        this.lift = new Lift((Motor) components[4], (Motor) components[5], voltageSensor);
        this.extension = new Extension((StepperServo) components[6], (StepperServo) components[7]);
        this.arm = new Arm((StepperServo) components[8], (StepperServo) components[9]);
        this.claw = new Claw((ContinuousServo) components[10], (ContinuousServo) components[11], colorSensor);
        this.climbWinch = new ClimbWinch((ContinuousServo) components[12], (ContinuousServo) components[13]);

        this.commands = new CommandMaster(this);
        this.hardwareMap = map;

        backLeft = (Motor) components[0];
        backRight = (Motor) components[1];
        frontLeft = (Motor) components[2];
        frontRight = (Motor) components[3];
    }

    // STATE MANAGEMENT AND TOGGLES
    public void toggleGamepiece() {
        if (mode == Gamepiece.SAMPLE) {
            mode = Gamepiece.SPECIMEN;
        } else {
            mode = Gamepiece.SAMPLE;
        }
    }

    public void toggleColorSensor() {
        activateSensor = !activateSensor;
    }

//AUTO OPERATIONS

    public void initSubsystems(){
        arm.runToPreset(Levels.INIT);
        lift.runToPreset(Levels.INIT);
        extension.runToPreset(Levels.INIT);
    }

    public Action autoSpecimen (boolean action) {
        return new SequentialAction(
                new InstantAction(() -> {
                    lift.runToPreset(Levels.INTAKE);
                }),
                new SleepAction(0.1),
                new InstantAction(() -> {
                    arm.runToPreset(Levels.INTERMEDIATE);
                    state = Levels.INTERMEDIATE;
                    claw.setPower(0);
                })
        );
    }

    public Action autoIntake (boolean action) {
        return new SequentialAction(
                new InstantAction(() -> {
                    lift.runToPreset(Levels.INTAKE);
                    extension.runToPosition(190);
                }),
                new SleepAction(0.3),
                new InstantAction(()->{
                    arm.runToPreset(Levels.INTAKE);
                    lift.slides1.resetEncoder();
                    claw.startIntake();
                    intaking = true;
                    state = Levels.INTAKE;}),
                new SleepAction(1),
                new InstantAction(() -> {
                    extension.runToPosition(226);
                })
        );
    }


    // INTAKE OPERATIONS

    public Action teleIntakePreset(boolean action) {
        return new SequentialAction(
                new InstantAction(() -> {
                    lift.runToPreset(Levels.INTAKE);
                    extension.runToPreset(Levels.INTAKE);
                }),
                new SleepAction(0.3),
                new InstantAction(() -> {
                    arm.runToPreset(Levels.INTAKE_INTERMEDIATE);
                    state = Levels.INTAKE_INTERMEDIATE;
                })
        );
    }

    public Action intakeDrop(SampleColors alliance) {
        if (activateSensor) {
            if (mode == Gamepiece.SAMPLE) {
                return new SequentialAction(
                        new InstantAction(() -> {
                            claw.startIntake();
                            arm.runToPreset(Levels.INTAKE);
                            lift.slides1.resetEncoder();
                            intaking = true;
                            state = Levels.INTAKE;
                        }),
                        commands.stopIntake(SampleColors.YELLOW, alliance)
                );
            } else {
                return new SequentialAction(
                        new InstantAction(() -> {
                            claw.startIntake();
                            arm.runToPreset(Levels.INTAKE);
                            lift.slides1.resetEncoder();
                            intaking = true;
                            state = Levels.INTAKE;
                        }),
                        commands.stopIntake(alliance)
                );
            }
        } else {
            return new SequentialAction(
                    new InstantAction(() -> {
                        claw.startIntake();
                        arm.runToPreset(Levels.INTAKE);
                        intaking = true;
                        state = Levels.INTAKE;
                    })
            );
        }
    }

    public void intermediatePreset() {
        arm.runToPreset(Levels.INTERMEDIATE);
        extension.runToPreset(Levels.INTERMEDIATE);
        lift.runToPreset(Levels.INTERMEDIATE);
        state = Levels.INTERMEDIATE;
    }

    public void stopIntake() {
        intaking = false;
        claw.stopIntake();
        intermediatePreset();
    }
    public Action stopIntakeAction() {
        return new InstantAction(()->{
            intaking = false;
            claw.stopIntake();
            intermediatePreset();
        } );

    }

    private ElapsedTime colorTimeout = new ElapsedTime();
    private ElapsedTime ejectTimeout = new ElapsedTime();
    boolean timerStarted = false;
    boolean ejectStarted = false;
    public boolean autoStopIntakeUpdate(SampleColors... colors) {
        int r = claw.smartStopDetect(colors);
        if (ejectStarted && ejectTimeout.time(TimeUnit.MILLISECONDS) >= 150) {
            ejectStarted = false;
            claw.setPower(1);
        } else if (!ejectStarted) {
            if (r == 0 && colorTimeout.time() > 0) {
                timerStarted = false;
                return true;
            } else if (r == 1 && timerStarted && colorTimeout.time() > 0.000) {
                while (colorTimeout.time() < 0.1) {
                    if (claw.smartStopDetect() != 1) {
                        return true;
                    }
                }
                stopIntake();
                return false;
            } else if (r == -1 && timerStarted && colorTimeout.time() > 0.050) {
                claw.setPower((float) -0.5);
                ejectTimeout.reset();
                ejectStarted = true;
                return true;
            } else if ((r == 1 || r == -1) && !timerStarted) {
                timerStarted = true;
                colorTimeout.reset();
            }
        }

        return true;
    }

    // DEPOSIT OPERATIONS
    public Action lowBasketAction() {
        return new InstantAction(()-> {
            arm.runToPreset(Levels.LOW_BASKET);
            extension.runToPreset(Levels.LOW_BASKET);
            lift.runToPreset(Levels.LOW_BASKET);
            state = Levels.LOW_BASKET;
        });
    }
    public Action highBasketAction() {
        return new InstantAction(()-> {
            arm.runToPreset(Levels.HIGH_BASKET);
            extension.runToPreset(Levels.HIGH_BASKET);
            lift.runToPreset(Levels.HIGH_BASKET);
            state = Levels.HIGH_BASKET;
        });
    }
    public Action highRung(boolean action) {
        return new InstantAction( () ->
        {arm.runToPreset(Levels.HIGH_RUNG);
            extension.runToPreset(Levels.HIGH_RUNG);
            lift.runToPreset(Levels.HIGH_RUNG);
            claw.setStall(true);
            state = Levels.HIGH_RUNG;}
        );
    }

    // OUTTAKE
    public Action outtakeSample(boolean action) {

        return new SequentialAction(
                claw.eject(true),
                new SleepAction(0.1),
                intermediateDepositPreset()
        );
    }
    public Action intermediateDepositPreset(){
        return new SequentialAction(
            new InstantAction(() ->{arm.runToPreset(Levels.INTERMEDIATE); extension.runToPreset(Levels.INTERMEDIATE);}),
            new SleepAction(0.1),
            new InstantAction(()->{lift.runToPreset(Levels.INTERMEDIATE);
                state = Levels.INTERMEDIATE;})
        );
    }

    public Action outtakeSpecimen(boolean action) {
        return new SequentialAction(
                claw.setStall(true,true),
                new SleepAction(0.5),
                claw.setStall(false,true)
        );
    }
    // DRIVE OPERATIONS
    public void setDrivePower(double x, double y, double rx) {
        double powerFrontLeft = y + x + rx;
        double powerFrontRight = y - x - rx;
        double powerBackLeft = (y - x + rx) * -1;
        double powerBackRight = (y + x - rx) * -1;

        if (Math.abs(powerFrontLeft) > 1 || Math.abs(powerBackLeft) > 1 ||
                Math.abs(powerFrontRight) > 1 || Math.abs(powerBackRight) > 1) {
            // Find the largest power
            double max;
            max = Math.max(Math.abs(powerFrontLeft), Math.abs(powerBackLeft));
            max = Math.max(Math.abs(powerFrontRight), max);
            max = Math.max(Math.abs(powerBackRight), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            powerFrontLeft /= max;
            powerBackLeft /= max;
            powerFrontRight /= max;
            powerBackRight /= max;
        }

        frontLeft.setSpeed((float) powerFrontLeft);
        frontRight.setSpeed((float) powerFrontRight);
        backLeft.setSpeed(-(float) powerBackLeft);
        backRight.setSpeed(-(float) powerBackRight);
    }

    // ENUMS
    public enum Gamepiece {
        SAMPLE,
        SPECIMEN
    }
}