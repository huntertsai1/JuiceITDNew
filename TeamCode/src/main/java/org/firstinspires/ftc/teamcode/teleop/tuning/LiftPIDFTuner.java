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
public class LiftPIDFTuner extends OpMode {
    private PIDController controller1;

    public static double p = 0.023, i = 0.00, d = 0.001;
    public static double f = 0.23;

    public static double MAX_ACCEL = 3000, MAX_VEL = 1500;
    public static boolean ACTIVATE_MP = true;
    public double voltageCompensation;

    public static int target = 0;
    private int oldTarget = 0;

    public ElapsedTime profileTimer = new ElapsedTime();
    public MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(0, 1, MAX_VEL, MAX_ACCEL);

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

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        profileTimer.reset();

        slides1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        controller1.setPID(p, i, d);
        int slides1Pos = slides1.getCurrentPosition();
        telemetry.addData("pos ", slides1Pos);

        if (oldTarget != target && target <= 100) {
            profile = MotionProfileGenerator.generateSimpleMotionProfile(slides1Pos, target, MAX_VEL, MAX_ACCEL);
            profileTimer.reset();
        }

        double pid1;
        double effectiveTarget = target;
        if ((ACTIVATE_MP && target <= 100) && slides1Pos != target) {
            effectiveTarget = profile.get(profileTimer.time(TimeUnit.MICROSECONDS)/1e6);
        }
        pid1 = controller1.calculate(slides1Pos, effectiveTarget);


        double ff = f;

        voltageCompensation = 13.2 / voltageSensor.getVoltage();

        double power1 = (pid1 + ff) * voltageCompensation;
        telemetry.addData("POWER ", power1);
        telemetry.addData("Voltage Compensation ", voltageCompensation);

        slides1.setPower(power1);
        slides2.setPower(power1);
        slides3.setPower(power1);

        oldTarget = target;

        telemetry.addData("POSITION ", slides1Pos);
        telemetry.addData("TARGET ", effectiveTarget);
        telemetry.addData("Motor 1 current", slides1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Motor 2 current", slides2.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Motor 3 current", slides3.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}