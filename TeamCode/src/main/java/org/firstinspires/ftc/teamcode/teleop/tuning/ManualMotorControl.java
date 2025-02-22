package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.lift.Lift;
import org.firstinspires.ftc.teamcode.util.hardware.ContinuousServo;
import org.firstinspires.ftc.teamcode.util.hardware.Motor;
import org.firstinspires.ftc.teamcode.util.hardware.StepperServo;

@TeleOp(group = "competition")
@Config
@Disabled
public class ManualMotorControl extends LinearOpMode {
//    public static double ARM_POS = 103;
//    public static double CLAW_SPEED1 = 0;
//    public static double CLAW_SPEED2 = 0;
//    public static double CLIMB_SPEED1 = 0;
//    public static double CLIMB_SPEED2 = 0;
//    public static double ELBOW_POS = 282;
//    public static double EXT_POS1 = 180;
//    public static double EXT_POS2 = 180;
    public static double LIFT_SPEED1 = 0;
    public static double LIFT_SPEED2 = 0;
    public static double LIFT_SPEED3 = 0;
    public static double LEFT_FRONT = 0;
    public static double RIGHT_FRONT = 0;
    public static double LEFT_BACK = 0;
    public static double RIGHT_BACK = 0;
    public static boolean LIFT_1_REVERSE = false;
    public static boolean LIFT_2_REVERSE = false;
    public static boolean LIFT_3_REVERSE = false;


    StepperServo ext1;
    StepperServo ext2;
    StepperServo arm;
    StepperServo elbow;
    ContinuousServo intake1;
    ContinuousServo intake2;
    ContinuousServo climb1;
    ContinuousServo climb2;

    Motor lift1;
    Motor lift2;
    Motor lift3;
    Motor leftFront;
    Motor rightFront;
    Motor leftBack;
    Motor rightBack;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ext1 = new StepperServo(0, "ext1", hardwareMap);
        ext2 = new StepperServo(0, "ext2", hardwareMap);
        arm = new StepperServo(0, "arm", hardwareMap);
        elbow = new StepperServo(0, "elbow", hardwareMap);
        intake1 = new ContinuousServo(0, "claw1", hardwareMap);
        intake2 = new ContinuousServo(0, "claw2", hardwareMap);
        climb1 = new ContinuousServo(0, "climb1", hardwareMap);
        climb2 = new ContinuousServo(0, "climb2", hardwareMap);
        lift1 = new Motor(0, "lift1", hardwareMap, true);
        lift2 = new Motor(0, "lift2", hardwareMap, false);
        lift3 = new Motor(0, "lift3", hardwareMap, true);
        leftBack = new Motor(0, "leftBack", hardwareMap, true);                //0 left odometer
        rightBack = new Motor(1, "rightBack", hardwareMap, false);             //1 right odometer
        leftFront = new Motor(2, "leftFront", hardwareMap, true);               //2 middle odometer
        rightFront = new Motor(3, "rightFront", hardwareMap, false);


        intake2.servo.setDirection(DcMotorSimple.Direction.REVERSE);
        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            if (LIFT_1_REVERSE) {
                lift1.motor.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                lift1.motor.setDirection(DcMotorSimple.Direction.FORWARD);
            }

            if (LIFT_2_REVERSE) {
                lift2.motor.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                lift2.motor.setDirection(DcMotorSimple.Direction.FORWARD);
            }

            if (LIFT_3_REVERSE) {
                lift3.motor.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                lift3.motor.setDirection(DcMotorSimple.Direction.FORWARD);
            }
//            ext1.setAngle((float) EXT_POS1);
//            ext2.setAngle((float) EXT_POS2);
//            arm.setAngle((float) ARM_POS);
//            elbow.setAngle((float) ELBOW_POS);
//            intake1.setSpeed((float) CLAW_SPEED1);
//            intake2.setSpeed((float) CLAW_SPEED2);
//            climb1.setSpeed((float) CLIMB_SPEED1);
//            climb2.setSpeed((float) CLIMB_SPEED2);
            lift1.setSpeed((float) LIFT_SPEED1);
            lift2.setSpeed((float) LIFT_SPEED2);
            lift3.setSpeed((float) LIFT_SPEED3);
            leftFront.setSpeed((float) LEFT_FRONT);
            rightFront.setSpeed((float) RIGHT_FRONT);
            leftBack.setSpeed((float) LEFT_BACK);
            rightBack.setSpeed((float) RIGHT_BACK);

        }
    }
}
