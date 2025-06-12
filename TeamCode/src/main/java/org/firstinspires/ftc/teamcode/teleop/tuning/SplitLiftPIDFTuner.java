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
//@Disabled
public class SplitLiftPIDFTuner extends OpMode {
    private PIDController controller1;
    private PIDController controller2;

    public static double p1 = 0.014, i1 = 0.00, d1 = 0.0007;
    public static double f1 = 0.15;

    public static double p2 = 0.014, i2 = 0.00, d2 = 0.0007;
    public static double f2 = 0.15;
    public double voltageCompensation;

    public static int target = 0;
    public int lastTarget = 0;
    public boolean isDown = false;

    private DcMotorEx slides1;
    private DcMotorEx slides2;
    private DcMotorEx slides3;
    private VoltageSensor voltageSensor;


    @Override
    public void init() {
        controller1 = new PIDController(p1, i1 , d1);
        controller2 = new PIDController(p2, i2 , d2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slides1 = hardwareMap.get(DcMotorEx.class, "lift1");
        slides2 = hardwareMap.get(DcMotorEx.class, "lift2");
        slides3 = hardwareMap.get(DcMotorEx.class, "lift3");

        slides1.setDirection(DcMotorSimple.Direction.REVERSE);
        slides2.setDirection(DcMotorSimple.Direction.REVERSE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        slides1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if (target < lastTarget) {
            isDown = true;
        } else if (target > lastTarget) {
            isDown = false;
        }
        lastTarget = target;

        controller1.setPID(p1, i1, d1);
        controller2.setPID(p2, i2, d2);
        int slides1Pos = slides1.getCurrentPosition();
        double pid1;

        voltageCompensation = 13.2 / voltageSensor.getVoltage();

        double power1;
        if (!isDown) {
            pid1 = controller1.calculate(slides1Pos, target);
            double ff = f1;
            power1 = (pid1 + ff) * voltageCompensation;
        } else {
            pid1 = controller2.calculate(slides1Pos, target);
            double ff = f2;
            power1 = (pid1 + ff) * voltageCompensation;
        }

        telemetry.addData("POWER ", power1);
        telemetry.addData("Voltage Compensation ", voltageCompensation);

        slides1.setPower(power1);
        slides2.setPower(power1);
        slides3.setPower(power1);

        telemetry.addData("POSITION ", slides1Pos);
        telemetry.addData("MODE", isDown ? "DOWN" : "UP");
        telemetry.addData("TARGET ", target);
        telemetry.addData("Motor 1 current", slides1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Motor 2 current", slides2.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Motor 3 current", slides3.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Motor 1 RPM", ((slides1.getVelocity()) * 60 / 28));
        telemetry.update();
    }
}