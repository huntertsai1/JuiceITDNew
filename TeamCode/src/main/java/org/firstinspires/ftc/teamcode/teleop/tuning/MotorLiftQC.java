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
import org.firstinspires.ftc.teamcode.util.control.MotionProfile;
import org.firstinspires.ftc.teamcode.util.control.MotionProfileGenerator;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp
//@Disabled
public class MotorLiftQC extends OpMode {
    private DcMotorEx lift1;
    private DcMotorEx lift2;
    private DcMotorEx lift3;
    public static double POWER1 = 0;

    public static boolean LIFT_1_REVERSE = true;
    public static boolean LIFT_2_REVERSE = true;
    public static boolean LIFT_3_REVERSE = false;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        lift3 = hardwareMap.get(DcMotorEx.class, "lift3");

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {

        if (LIFT_1_REVERSE) {
            lift1.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            lift1.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        if (LIFT_2_REVERSE) {
            lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            lift2.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        if (LIFT_3_REVERSE) {
            lift3.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            lift3.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        int liftPos1 = lift1.getCurrentPosition();

        lift1.setPower(POWER1);
        lift2.setPower(POWER1);
        lift3.setPower(POWER1);

        telemetry.addData("Motor 1 current", lift1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Motor 2 current", lift2.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Motor 3 current", lift3.getCurrent(CurrentUnit.AMPS));

        telemetry.addData("RPM", ((lift1.getVelocity())*60 / 28));
        telemetry.addData("POS", (liftPos1));


        telemetry.update();
    }
}