package org.firstinspires.ftc.teamcode;

// IMPORT SUBSYSTEMS

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.commands.CommandMaster;
import org.firstinspires.ftc.teamcode.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.climb.ClimbWinch;
import org.firstinspires.ftc.teamcode.subsystems.extension.Extension;
import org.firstinspires.ftc.teamcode.subsystems.lift.Lift;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.util.enums.AllianceColor;
import org.firstinspires.ftc.teamcode.util.enums.ClimbType;
import org.firstinspires.ftc.teamcode.util.hardware.BrushlandColorSensor;
import org.firstinspires.ftc.teamcode.util.hardware.Component;
import org.firstinspires.ftc.teamcode.util.hardware.ContinuousServo;
import org.firstinspires.ftc.teamcode.util.enums.Levels;
import org.firstinspires.ftc.teamcode.util.hardware.Motor;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;
import org.firstinspires.ftc.teamcode.util.hardware.StepperServo;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.Executor;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

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
    public boolean l3ClimbOverride = false;
    public Levels state = Levels.INIT;
    public Gamepiece mode = Gamepiece.SAMPLE;
    public SampleColors targetColor = SampleColors.YELLOW;
    public ClimbType climbMode = ClimbType.LEVEL_3;
    Motor backLeft;
    Motor backRight;
    Motor frontLeft;
    Motor frontRight;

    public IMU imu;


    public Robot(HardwareMap map, boolean auton){
        this.auton = auton;

        //Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        //this.drive = new KalmanDrive(map, PoseKeeper.get(), limelight);

//        this.cv = new CVMaster(map);
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

                new ContinuousServo(0, "climb1", map, "climb1Encoder"),//12
                new ContinuousServo(1, "climb2", map, "climb2Encoder") //13
        };

        VoltageSensor voltageSensor = map.voltageSensor.iterator().next();
        //RevColorSensorV3 colorSensor = map.get(RevColorSensorV3.class, "colorSensor");
        BrushlandColorSensor colorSensor = new BrushlandColorSensor(0, "color", map);

        // INIT SUBSYSTEMS

        this.lift = new Lift((Motor) components[4], (Motor) components[5], voltageSensor);
        this.extension = new Extension((StepperServo) components[6], (StepperServo) components[7]);
        this.arm = new Arm((StepperServo) components[8], (StepperServo) components[9]);
        this.claw = new Claw((ContinuousServo) components[10], (ContinuousServo) components[11], colorSensor);
        this.climbWinch = new ClimbWinch((ContinuousServo) components[12], (ContinuousServo) components[13]);

        //this.imu = hardwareMap.get(IMU.class, "imu");

        this.commands = new CommandMaster(this);
//        this.cv = new CVMaster(limelight, hardwareMap.get(WebcamName.class, "Webcam 1"));
        //this.cv = new CVMaster(limelight,null);
        this.hardwareMap = map;

        backLeft = (Motor) components[0];
        backRight = (Motor) components[1];
        frontLeft = (Motor) components[2];
        frontRight = (Motor) components[3];
    }

    public Action initSubsystems(boolean action){
        return new InstantAction(()->{
            arm.runToPreset(Levels.INIT);
            lift.runToPreset(Levels.INIT);
            extension.runToPreset(Levels.INIT);
        });
    }
    public void initSubsystems(){
        arm.runToPreset(Levels.INIT);
        lift.runToPreset(Levels.INIT);
        extension.runToPreset(Levels.INIT);
    }

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

    public void toggleGamepiece(Gamepiece p) {
        mode = p;
    }

    public void toggleGamepieceColor(AllianceColor allianceColor) {
        if ((targetColor == SampleColors.BLUE || targetColor == SampleColors.RED) && mode == Gamepiece.SAMPLE) {
            targetColor = SampleColors.YELLOW;
        } else {
            if (allianceColor == AllianceColor.RED) {
                targetColor = SampleColors.RED;
            } else {
                targetColor = SampleColors.BLUE;
            }
        }
    }

    public void toggleClimbMode() {
        if (climbMode == ClimbType.LEVEL_2) {
            climbMode = ClimbType.LEVEL_3;
        } else if (climbMode == ClimbType.LEVEL_3) {
            climbMode = ClimbType.LEVEL_2;
        }
    }

    // INTAKE PRESETS

    public void intakePreset() {
        lift.runToPreset(Levels.INTAKE);
        arm.runToPreset(Levels.INTAKE_INTERMEDIATE);
        extension.runToPreset(Levels.INTAKE);

        new Thread(() -> {
            sleep(3000);
            arm.runToPreset(Levels.INTAKE);
            claw.startIntake();
            intaking = true;
            state = Levels.INTAKE;
        }).start();
    }

    public void intakePreset(double extTicks) {
        lift.runToPreset(Levels.INTAKE);
        arm.runToPreset(Levels.INTAKE_INTERMEDIATE);
        //TODO: CONVERT FROM INCHES TO TICKS
        extension.runToPosition((float) extTicks);

        new Thread(() -> {
            sleep(3000);
            arm.runToPreset(Levels.INTAKE);
            claw.startIntake();
            intaking = true;
            state = Levels.INTAKE;
        }).start();
    }

    public Action intakePreset(double extTicks, boolean action) {
        return new SequentialAction(
                new InstantAction(() -> {
                    lift.runToPreset(Levels.INTAKE);
                    extension.runToPreset(Levels.INTAKE);
                }),
                new SleepAction(0.5),
                new InstantAction(() -> {
                    arm.runToPreset(Levels.INTAKE_INTERMEDIATE);
                    lift.slides1.resetEncoder();
                    state = Levels.INTAKE_INTERMEDIATE;
                }),
                new SleepAction(0.2),
                new InstantAction(()->{
                    claw.startIntake();
                    arm.runToPreset(Levels.INTAKE);
                    intaking = true;
                    state = Levels.INTAKE;})

        );
    }

    public Action retractedIntakePreset(boolean action) {
        return new SequentialAction(
                new InstantAction(() -> {
                    lift.runToPreset(Levels.INTAKE);
                    extension.runToPosition(200);
                }),
                new SleepAction(0.5),
                new InstantAction(() -> {
                    arm.runToPreset(Levels.INTAKE_INTERMEDIATE);
                    lift.slides1.resetEncoder();
                    state = Levels.INTAKE_INTERMEDIATE;
                }),
                new SleepAction(0.2),
                new InstantAction(()->{
                    claw.startIntake();
                    arm.runToPreset(Levels.INTAKE);
                    intaking = true;
                    state = Levels.INTAKE;})

        );
    }

    public Action teleIntakePreset(boolean action) {
        return new SequentialAction(
                new InstantAction(() -> {
                    lift.runToPreset(Levels.INTAKE);
                    extension.runToPreset(Levels.INTAKE);
                }),
                new SleepAction(0.5),
                new InstantAction(() -> {
                    arm.runToPreset(Levels.INTAKE_INTERMEDIATE);
                    lift.slides1.resetEncoder();
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

    public void autonObParkPreset() {
        lift.runToPreset(Levels.INTAKE);
        arm.runToPreset(Levels.INTAKE_INTERMEDIATE);
        extension.runToPosition(100);
    }

    public void intermediatePreset() {
        arm.runToPreset(Levels.INTERMEDIATE);
        extension.runToPreset(Levels.INTERMEDIATE);
        lift.runToPreset(Levels.INTERMEDIATE);
        state = Levels.INTERMEDIATE;
    }

    public void scanForTargetsPreset() {
        arm.runToPreset(Levels.INTERMEDIATE);
        extension.runToPreset(Levels.INTERMEDIATE);
        lift.runToPreset(Levels.INTERMEDIATE);
        state = Levels.LOCATING_TARGETS;
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
    boolean timerStarted = false;
    public boolean autoStopIntakeUpdate(SampleColors... colors) {
        int r = claw.smartStopDetect(colors);
        if (r == 0 && colorTimeout.time() > 0) {
            timerStarted = false;
            return true;
        } else if (r == 1 && timerStarted && colorTimeout.time() > 0.000) {
            while (colorTimeout.time() < 0.1){
                if (claw.smartStopDetect() != 1){
                    return true;
                }
            }
            stopIntake();
            return false;
        } else if (r == -1 && timerStarted && colorTimeout.time() > 0.000) {
            claw.ejectOps();
            return true;
        } else if ((r == 1 || r == -1) && !timerStarted) {
            timerStarted = true;
            colorTimeout.reset();
        }

        return true;
    }

    // DEPOSIT PRESETS
    public void lowBasket() {
        arm.runToPreset(Levels.LOW_BASKET);
        extension.runToPreset(Levels.LOW_BASKET);
        lift.runToPreset(Levels.LOW_BASKET);
        state = Levels.LOW_BASKET;
    }

    public Action lowBasketAction() {
        return new InstantAction(()-> {
            arm.runToPreset(Levels.LOW_BASKET);
            extension.runToPreset(Levels.LOW_BASKET);
            lift.runToPreset(Levels.LOW_BASKET);
            state = Levels.LOW_BASKET;
        });
    }

    public void highBasket() {
        arm.runToPreset(Levels.HIGH_BASKET);
        extension.runToPreset(Levels.HIGH_BASKET);
        lift.runToPreset(Levels.HIGH_BASKET);
        state = Levels.HIGH_BASKET;
    }
    public Action highBasketAction() {
        return new InstantAction(()-> {
            arm.runToPreset(Levels.HIGH_BASKET);
            extension.runToPreset(Levels.HIGH_BASKET);
            lift.runToPreset(Levels.HIGH_BASKET);
            state = Levels.HIGH_BASKET;
        });
    }

    public void lowRung() {
        arm.runToPreset(Levels.LOW_RUNG);
        extension.runToPreset(Levels.LOW_RUNG);
        lift.runToPreset(Levels.LOW_RUNG);
        state = Levels.LOW_RUNG;
    }

    public void highRung() {
        arm.runToPreset(Levels.HIGH_RUNG);
        extension.runToPreset(Levels.HIGH_RUNG);
        lift.runToPreset(Levels.HIGH_RUNG);
        state = Levels.HIGH_RUNG;
    }
    public Action highRung(boolean action) {
        return new InstantAction( () ->
        {arm.runToPreset(Levels.HIGH_RUNG);
            extension.runToPreset(Levels.HIGH_RUNG);
            lift.runToPreset(Levels.HIGH_RUNG);
            state = Levels.HIGH_RUNG;}
        );
    }

    public void preloadHighRung() {
        arm.runToPosition(0);
        extension.runToPosition(0);
        lift.runToPosition(0);
        state = Levels.HIGH_RUNG;
    }

    /**
     * <h1>Drop preload to the goat <u><b>DITA RAJEEV</b></u></h1>
     */
    public void preloadDropPreset() {
        arm.runToPreset(Levels.INTAKE);
        extension.runToPosition(0);
        lift.runToPreset(Levels.INTAKE);
        state = Levels.INTERMEDIATE;
    }


    public void teleDepositPreset() {
        if (mode == Gamepiece.SAMPLE) {
            highBasket();
        } else {
            highRung();
        }
    }

    // OUTTAKE

    public void outtakeSample() {
        claw.eject();
    }

    public Action outtakeSample(boolean action) {

        return new SequentialAction(
                claw.eject(true),
                new InstantAction(
                        this::intermediatePreset)
        );
    }

    public void outtakeSpecimen() {
        claw.stall();
        lift.runToPosition(lift.getPos() - 10);
        new Thread(() -> {
            sleep(50);
            claw.stall();
        }).start();
    }

    public Action outtakeSpecimen(boolean action) {
        return new SequentialAction(
                claw.setStall(true,true),
                new SleepAction(0.5),
                claw.setStall(false,true)
        );
    }

    public void outtakeSpecimenPreload() {
        claw.stall();
        lift.runToPosition(lift.getPos() - 10);
        new Thread(() -> {
            sleep(50);
            lift.runToPosition(0);
            claw.stall()
            ;
        }).start();
    }

    public void smartOuttake() {
        if (state == Levels.LOW_BASKET || state == Levels.HIGH_BASKET) {
            outtakeSample();
            intermediatePreset();
            state = Levels.INTERMEDIATE;
        } else if (state == Levels.LOW_RUNG || state == Levels.HIGH_RUNG) {
            outtakeSpecimen();
            intermediatePreset();
            state = Levels.INTERMEDIATE;
        }
    }

    public Action smartOuttake(boolean action) {
        if (state == Levels.LOW_BASKET || state == Levels.HIGH_BASKET) {
            return new SequentialAction(
                    outtakeSample(true),
                    new InstantAction(() -> {
                        intermediatePreset();
                        state = Levels.INTERMEDIATE;
                    })
            );
        } else if (state == Levels.LOW_RUNG || state == Levels.HIGH_RUNG) {
            return new SequentialAction(
                    outtakeSpecimen(true),
                    new InstantAction(() -> {
                        intermediatePreset();
                        state = Levels.INTERMEDIATE;
                    })
            );
        }
        return new NullAction();
    }


    public Action primeClimb() {
        if (climbMode == ClimbType.LEVEL_3) {
            return new SequentialAction(
                    climbWinch.up(),
                    new InstantAction(() -> {
                        lift.runToPreset(Levels.CLIMB_EXTENDED);
                        state = Levels.CLIMB_PRIMED;
                    })
            );
        } else {
            return new SequentialAction(
                    climbWinch.up(),
                    new InstantAction(() -> state = Levels.CLIMB_PRIMED)
            );
        }
    }

    public Action startClimbL3() {
        return new InstantAction(() -> {
            lift.runToPreset(Levels.CLIMB_RETRACTED);
            state = Levels.CLIMB_L3;
        });
    }

    //DRIVE
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

    public void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public enum Gamepiece {
        SAMPLE,
        SPECIMEN
    }

    public static double normalizeRadians(double angle) {
        angle = angle % (2 * Math.PI);
        if (angle < 0) {
            angle += 2 * Math.PI;
        }
        return angle;
    }
}