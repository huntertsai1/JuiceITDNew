package org.firstinspires.ftc.teamcode;

// IMPORT SUBSYSTEMS

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.util.CommandMaster;
import org.firstinspires.ftc.teamcode.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.climb.ClimbWinch;
import org.firstinspires.ftc.teamcode.subsystems.extension.Extension;
import org.firstinspires.ftc.teamcode.subsystems.lift.Lift;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.util.enums.ClimbType;
import org.firstinspires.ftc.teamcode.util.hardware.BrushlandColorSensor;
import org.firstinspires.ftc.teamcode.util.hardware.Component;
import org.firstinspires.ftc.teamcode.util.hardware.ContinuousServo;
import org.firstinspires.ftc.teamcode.util.enums.Levels;
import org.firstinspires.ftc.teamcode.util.hardware.GoBildaLEDIndicator;
import org.firstinspires.ftc.teamcode.util.hardware.Motor;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;
import org.firstinspires.ftc.teamcode.util.hardware.StepperServo;

public class Robot {

    // SUBSYSTEM DECLARATIONS
    public Component[] components;
    public Lift lift;
    public Extension extension;
    public Arm arm;
    public Claw claw;
    public ClimbWinch climbWinch;
    public Sweeper sweeper;

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
    public Motor backLeft;
    public Motor backRight;
    public Motor frontLeft;
    public Motor frontRight;
    public GoBildaLEDIndicator blinky;

    public Robot(HardwareMap map, boolean auton){
        this.auton = auton;

        this.components = new Component[]{
                new Motor(0, "leftBack", map, true),                //0 left odometer
                new Motor(1, "rightBack", map, false),              //1 right odometer
                new Motor(2, "leftFront", map, true),               //2 middle odometer
                new Motor(3, "rightFront", map, false),             //3

                new Motor(0, "lift1", map, true),                  //4
                new Motor(1, "lift2", map, true),                  //5
                new Motor(2, "lift3", map, false),                  // 6

                new StepperServo(2, "ext1", map),    //7
                new StepperServo(3, "ext2", map),    //8

                new StepperServo(4, "arm", map),      //9

                new StepperServo(5, "elbow", map),  //10
                new ContinuousServo(1, "claw1", map),                     //11
                new ContinuousServo(5, "claw2", map),                     //12

                new ContinuousServo(0, "climb1", map, "climb1Encoder", false),//13
                new ContinuousServo(1, "climb2", map, "climb2Encoder", true), //14

                new StepperServo(0, "sweeper", map)         //15
        };

//        RevColorSensorV3 colorSensor = map.get(RevColorSensorV3.class, "colorSensor");
        BrushlandColorSensor colorSensor1 = new BrushlandColorSensor(0, "colorSensorHead", map);
        BrushlandColorSensor colorSensor2 = new BrushlandColorSensor(0, "colorSensorTail", map, true);
        blinky = new GoBildaLEDIndicator(0, "blinky", map);
        // INIT SUBSYSTEMS

        this.lift = new Lift((Motor) components[4], (Motor) components[5], (Motor) components[6]);
        this.extension = new Extension((StepperServo) components[7], (StepperServo) components[8]);
        this.arm = new Arm((StepperServo) components[9], (StepperServo) components[10]);
        this.claw = new Claw((ContinuousServo) components[11], (ContinuousServo) components[12], colorSensor1, colorSensor2);
        this.climbWinch = new ClimbWinch((ContinuousServo) components[13], (ContinuousServo) components[14]);
        this.sweeper = new Sweeper((StepperServo) components[15]);

        this.commands = new CommandMaster(this);
        this.hardwareMap = map;

        backLeft = (Motor) components[0];
        backRight = (Motor) components[1];
        frontLeft = (Motor) components[2];
        frontRight = (Motor) components[3];
    }

    // STATE MANAGEMENT AND TOGGLES
    public Gamepiece toggleGamepiece() {
        if (mode == Gamepiece.SAMPLE) {
            mode = Gamepiece.SPECIMEN;
            blinky.set(GoBildaLEDIndicator.Colors.VIOLET, GoBildaLEDIndicator.Animation.SLOW_BLINK);
        } else {
            mode = Gamepiece.SAMPLE;
            blinky.set(GoBildaLEDIndicator.Colors.BLUE, GoBildaLEDIndicator.Animation.SLOW_BLINK);
        }
        return mode;
    }

    public void toggleColorSensor() {
        activateSensor = !activateSensor;
    }

//AUTO OPERATIONS

    public void initSubsystems(){
        arm.runToPreset(Levels.INIT);
        lift.runToPreset(Levels.INIT);
        extension.runToPreset(Levels.INIT);
        sweeper.setPosition(92);
        blinky.set(GoBildaLEDIndicator.Colors.YELLOW, GoBildaLEDIndicator.Animation.SOLID);
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

    public Action autoSpecIntake(boolean action) {
        return new SequentialAction(
                new InstantAction(() -> {
                    lift.runToPreset(Levels.INTAKE);
                }),
                new SleepAction(0.2),
                new InstantAction(() -> {
                    extension.runToPosition(240);
                }),
                new SleepAction(0.3),
                new InstantAction(()->{
                    arm.runToPreset(Levels.INTAKE);
                    claw.eject();
                    intaking = true;
                    state = Levels.INTAKE;}),
                new SleepAction(1.5),
                new InstantAction(() -> {
                    extension.runToPosition(280);
                    lift.lift1.resetEncoder();
                })
                );
    }

    public Action autoTeleIntakePrime() {
        return new SequentialAction(
                new InstantAction(() -> {
                    lift.runToPreset(Levels.INTAKE);
                }),
                new SleepAction(0.4),
                new InstantAction(() -> {
                    extension.runToPosition(190);
                }),
                new SleepAction(0.3),
                new InstantAction(()->{
                    arm.runToPreset(Levels.INTAKE);
                    claw.eject();
                    intaking = true;
                    state = Levels.INTAKE;})
        );
    }

    public Action autoTeleIntakeBoom() {
        return new SequentialAction(
                new InstantAction(() -> {
                    extension.runToPosition(270);
                    lift.lift1.resetEncoder();
                })
        );
    }

    public ElapsedTime actionRunning = new ElapsedTime();

    public Action autoBucketIntake (boolean action) {
        return new SequentialAction(
                new InstantAction(() -> {
                    lift.runToPreset(Levels.INTAKE);
                    extension.runToPosition(160);
                }),
                new SleepAction(0.3),
                new InstantAction(()->{
                    arm.runToPreset(Levels.INTAKE);
                    lift.lift1.resetEncoder();
                    claw.setPower(0.8F);
                    intaking = true;
                    state = Levels.INTAKE;}),
                new SleepAction(0.3), // DELAY BETWEEN ARM DROPPING AND EXTENSION FULLY EXTENDING, EDIT IF NEEDED
                new InstantAction(() -> {
                    extension.runToPosition(250);
                }),
                new SleepAction(1),
                new InstantAction(() -> {
                    extension.runToPosition(285);
                    intaking = true;
                    state = Levels.INTAKE;
                    claw.intakeStatus = 0;
                }),commands.stopIntake(SampleColors.YELLOW)
        );
    }

    public Action autoBucketIntakeTHIRD (boolean action) {
        return new SequentialAction(
                new InstantAction(() -> {
                    lift.runToPreset(Levels.INTAKE);
                    extension.runToPosition(135);
                }),
                new SleepAction(0.3),
                new InstantAction(()->{
                    arm.runToPreset(Levels.INTAKE);
                    lift.lift1.resetEncoder();
                    claw.setPower(0.8F);
                    intaking = true;
                    state = Levels.INTAKE;}),
                new SleepAction(0.3), // DELAY BETWEEN ARM DROPPING AND EXTENSION FULLY EXTENDING, EDIT IF NEEDED
                new InstantAction(() -> {
                    extension.runToPosition(250);
                }),
                new SleepAction(1),
                new InstantAction(() -> {
                    extension.runToPosition(280);
                })
        );
    }


    // INTAKE OPERATIONS

    public Action teleIntakePreset(boolean action) {
        return new SequentialAction(
                new InstantAction(() -> {
                    lift.runToPreset(Levels.INTAKE);
                    extension.runToPreset(Levels.INTAKE);
                    if (mode == Gamepiece.SAMPLE) {
                        //blinky.set(GoBildaLEDIndicator.Colors.YELLOW, GoBildaLEDIndicator.Animation.SLOW_BLINK);
                    } else {
                        //blinky.set(GoBildaLEDIndicator.Colors.INDIGO, GoBildaLEDIndicator.Animation.SLOW_BLINK);
                    }
                }),
                new SleepAction(0.1),
                new InstantAction(() -> {
                    arm.runToPreset(Levels.INTAKE_INTERMEDIATE);
                    state = Levels.INTAKE_INTERMEDIATE;
                })
        );
    }

    public void teleIntakePreset() {
        lift.runToPreset(Levels.INTAKE);
        extension.runToPreset(Levels.INTAKE);
//        if (mode == Gamepiece.SAMPLE) {
//            //blinky.set(GoBildaLEDIndicator.Colors.YELLOW, GoBildaLEDIndicator.Animation.SLOW_BLINK);
//        } else {
//            //blinky.set(GoBildaLEDIndicator.Colors.INDIGO, GoBildaLEDIndicator.Animation.SLOW_BLINK);
//        }
        arm.runToPreset(Levels.INTAKE_INTERMEDIATE);
        state = Levels.INTAKE_INTERMEDIATE;
    }

    public Action intakeDrop(SampleColors alliance, Gamepad gamepad) {
        if (activateSensor) {
            if (mode == Gamepiece.SAMPLE) {
                return new SequentialAction(
                        new InstantAction(() -> {
                            claw.startIntake();
                            arm.runToPreset(Levels.INTAKE);
                            lift.lift1.resetEncoder();
                            intaking = true;
                            state = Levels.INTAKE;
                            claw.intakeStatus = 0;
                            //blinky.set(GoBildaLEDIndicator.Colors.YELLOW, GoBildaLEDIndicator.Animation.BLINK);
                        }),
                        commands.stopIntake(gamepad, SampleColors.YELLOW, alliance)
                );
            } else {
                return new SequentialAction(
                        new InstantAction(() -> {
                            claw.startIntake();
                            arm.runToPreset(Levels.INTAKE);
                            lift.lift1.resetEncoder();
                            intaking = true;
                            claw.intakeStatus = 0;
                            state = Levels.INTAKE;
//                            blinky.set(GoBildaLEDIndicator.Colors.INDIGO, GoBildaLEDIndicator.Animation.BLINK);
                        }),
                        commands.stopIntake(gamepad, alliance)
                );
            }
        } else {
            return new SequentialAction(
                    new InstantAction(() -> {
                        claw.startIntake();
                        arm.runToPreset(Levels.INTAKE);
                        lift.lift1.resetEncoder();
                        intaking = true;
                        state = Levels.INTAKE;
//                        if (mode == Gamepiece.SAMPLE) {
//                            blinky.set(GoBildaLEDIndicator.Colors.YELLOW, GoBildaLEDIndicator.Animation.BLINK);
//                        } else {
//                            blinky.set(GoBildaLEDIndicator.Colors.INDIGO, GoBildaLEDIndicator.Animation.BLINK);
//                        }
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
        afterAction.reset();
        intermediatePreset();
//        blinky.set(GoBildaLEDIndicator.Colors.BLUE, GoBildaLEDIndicator.Animation.THREE_BLIPS);
    }
    public Action stopIntakeAction() {
        return new InstantAction(()->{
            intaking = false;
            claw.intakeStatus = 16236;
            claw.stopIntake();
            intermediatePreset();
//            blinky.set(GoBildaLEDIndicator.Colors.BLUE, GoBildaLEDIndicator.Animation.THREE_BLIPS);
        } );
    }

    public Action specIntakeStop() {
        return new SequentialAction(
                new InstantAction(()->{
            intaking = false;
            claw.stopIntake();
            lift.runToPreset(Levels.INTERMEDIATE);
//            blinky.set(GoBildaLEDIndicator.Colors.BLUE, GoBildaLEDIndicator.Animation.THREE_BLIPS);
        } ),
                new SleepAction(0.1),
                new InstantAction(() -> {
                    extension.runToPreset(Levels.INTERMEDIATE);
                    arm.runToPreset(Levels.INTERMEDIATE);
                    state = Levels.INTERMEDIATE;
                }));
    }

    public ElapsedTime timeToAction = new ElapsedTime();
    public ElapsedTime afterAction = new ElapsedTime();
    boolean timerStarted = false;
    boolean ejectStarted = false;
    public boolean autoStopIntakeUpdate(Gamepad gamepad, SampleColors... colors) {
        int r;
        if (activateSensor) {
            r = claw.smartStopDetect(colors);
        } else {
            r = claw.smartStopDetectFAIL(false, colors);
        }
        System.out.println("r: " + r);
        if (r == 0) {
            claw.startIntake();
            return true;
        } else if (r == 1) {
            timeToAction.reset();
            afterAction.reset();
            gamepad.rumble(250);
            stopIntake();
            return false;
        } else if (r == -1) {
            claw.eject();
            return true;
        } else if (r == 16236) {
            //troll status code
            return false;
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
            //blinky.setPreset(Levels.LOW_BASKET);
        });
    }
    public Action highBasketAction() {
        return new InstantAction(()-> {
            arm.runToPreset(Levels.HIGH_BASKET);
            extension.runToPreset(Levels.HIGH_BASKET);
            lift.runToPreset(Levels.HIGH_BASKET);
            state = Levels.HIGH_BASKET;
            //blinky.setPreset(Levels.HIGH_BASKET);
        });
    }
    public Action autoHighBasketAction() {
        return new SequentialAction(
                new InstantAction(() -> {
                    extension.runToPreset(Levels.HIGH_BASKET);
                    lift.runToPreset(Levels.HIGH_BASKET);
                    arm.runToPreset(Levels.INTERMEDIATE);
                }),
                new SleepAction(1.4),
                new InstantAction(() -> {
                    arm.runToPreset(Levels.HIGH_BASKET);
                    state = Levels.HIGH_BASKET;
                })
        );
    }
    public Action highRung(boolean action) {
        return new ParallelAction(
                new InstantAction(
                        () -> {
                            arm.runToPreset(Levels.HIGH_RUNG);
                            extension.runToPreset(Levels.HIGH_RUNG);
                            lift.runToPreset(Levels.HIGH_RUNG);
                            state = Levels.HIGH_RUNG;
                            //blinky.setPreset(Levels.HIGH_RUNG);
                }),
                new SequentialAction(
                        new InstantAction(() -> claw.setPower(1)),
                        new SleepAction(0.05),
                        new InstantAction(() -> claw.setPower(0)),
                        new SleepAction(0.5),
                        new InstantAction(() -> claw.setPower(-0.4F)),
                        new SleepAction(0.05),
                        new InstantAction(() -> claw.setStall(true))
                )
        );
    }

    public Action autoCenterSpecimen(boolean action) {
        return new SequentialAction(
                new InstantAction(() -> claw.setPower(1)),
                new SleepAction(0.05),
                new InstantAction(() -> claw.setPower(0)),
                new SleepAction(0.5),
                new InstantAction(() -> claw.setPower(-0.4F)),
                new SleepAction(0.05),
                new InstantAction(() -> claw.setStall(true))
        );
    }

    public Action highRungAuto(boolean action) {
        return new SequentialAction(
            new InstantAction(() -> claw.setPower(-0.3F)),
            new InstantAction(() -> {
                extension.runToPreset(Levels.HIGH_RUNG);
                arm.runToPreset(Levels.HIGH_RUNG);
            }),
            new SleepAction(0.2),
            new InstantAction(() -> {
                claw.setStall(true);
                lift.runToPreset(Levels.HIGH_RUNG);
                state = Levels.HIGH_RUNG;
            })
        );
    }

    // OUTTAKE
    public Action outtakeSample(boolean action) {
        //blinky.set(GoBildaLEDIndicator.Colors.OFF, GoBildaLEDIndicator.Animation.SOLID);
        return new SequentialAction(
                claw.ejectSample(true),
                new SleepAction(0.14),
                intermediateDepositPreset()
        );
    }
    public Action intermediateDepositPreset(){
        return new SequentialAction(
            new InstantAction(() ->{arm.runToPreset(Levels.INTERMEDIATE); extension.runToPreset(Levels.INTERMEDIATE);}),
            new SleepAction(0.2),
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

    // AUTOMATIC DRIVING
    public Action teleAutoIntakePrime (boolean action) {
        return new SequentialAction(
                new InstantAction(() -> {
                    lift.runToPreset(Levels.INTAKE);
                    extension.runToPosition(190);
                }),
                new SleepAction(0.3),
                new InstantAction(()->{
                    arm.runToPreset(Levels.INTAKE);
                    lift.lift1.resetEncoder();
                    claw.eject();
                    intaking = true;
                    state = Levels.INTAKE;})
        );
    }

    public Action teleAutoIntake() {
        return new InstantAction(() -> {
            extension.runToPosition(230);
        });
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