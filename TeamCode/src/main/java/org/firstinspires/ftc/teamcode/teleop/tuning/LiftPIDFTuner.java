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

@Config
@TeleOp
@Disabled
public class LiftPIDFTuner extends OpMode {
    private PIDController controller1;

    public static double p = 0.0125, i = 0.00, d = 0.0004;
    public static double f = 0.15;
    public static int target = 0;

    private DcMotorEx slides1;
    private DcMotorEx slides2;
    private DcMotorEx slides3;
    private VoltageSensor voltageSensor;


    @Override
    public void init() {
        controller1 = new PIDController(p, i , d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slides1 = hardwareMap.get(DcMotorEx.class, "lift1");
        slides2 = hardwareMap.get(DcMotorEx.class, "lift2");
        slides3 = hardwareMap.get(DcMotorEx.class, "lift3");

        slides1.setDirection(DcMotorSimple.Direction.REVERSE);
        slides2.setDirection(DcMotorSimple.Direction.REVERSE);

        slides1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        controller1.setPID(p, i, d);
        int slides1Pos = slides1.getCurrentPosition();
        double pid1;

        pid1 = controller1.calculate(slides1Pos, target);

        double ff = f;

        double power1 = (pid1 + ff);

        telemetry.addData("POWER ", power1);

        slides1.setPower(power1);
        slides2.setPower(power1);
        slides3.setPower(power1);

        telemetry.addData("POSITION ", slides1Pos);
        telemetry.addData("TARGET ", target);
        telemetry.addData("Motor 1 current", slides1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Motor 2 current", slides2.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Motor 3 current", slides3.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Motor 1 RPM", ((slides1.getVelocity())*60 / 28));
        telemetry.update();
    }
}