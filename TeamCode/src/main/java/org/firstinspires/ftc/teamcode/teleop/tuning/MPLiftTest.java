package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.lift.Lift;
import org.firstinspires.ftc.teamcode.util.control.MotionProfile;
import org.firstinspires.ftc.teamcode.util.control.MotionProfileGenerator;
import org.firstinspires.ftc.teamcode.util.hardware.Motor;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp
//@Disabled
public class MPLiftTest extends OpMode {
    Lift lift;
    boolean lBumper = false;
    boolean rBumper = false;

    public static int HIGH_POS = 1000;
    public static int LOW_POS = 0;

    public static int RUN = 0;
    public static int STOP = 0;
    public static boolean nand = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift = new Lift(new Motor(0, "lift1", hardwareMap, false), new Motor(0, "lift2", hardwareMap, false), new Motor(0, "lift3", hardwareMap, false), hardwareMap.voltageSensor.iterator().next());
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper && !rBumper) {
            lift.runToPosition(HIGH_POS);
        }

        if (gamepad1.left_bumper && !lBumper) {
            lift.runToPosition(LOW_POS);
        }

        lBumper = gamepad1.left_bumper;
        rBumper = gamepad1.right_bumper;
        if (Double.isNaN(lift.power1)) {
            nand = true;
        }

        lift.update();

        telemetry.addData("POSITION ", lift.getPos());
        telemetry.addData("TARGET", lift.effectiveTarget);
        telemetry.addData("motor power", lift.power1);
        telemetry.addData("nand", nand);
//        telemetry.addData("Motor 1 current", lift1.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("Motor 2 current", lift2.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("Motor 3 current", lift3.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}