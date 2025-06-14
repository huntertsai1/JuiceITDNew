package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.util.CommandMaster;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.extension.Extension;
import org.firstinspires.ftc.teamcode.subsystems.lift.Lift;
import org.firstinspires.ftc.teamcode.util.enums.Levels;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;
import org.firstinspires.ftc.teamcode.util.hardware.BrushlandColorSensor;
import org.firstinspires.ftc.teamcode.util.hardware.ContinuousServo;

import java.util.ArrayList;
import java.util.List;

@TeleOp(group = "competition")
@Config
//@Disabled
public class IntakeTest extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, false);
        List<Action> actionsQueue = new ArrayList<>();
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        boolean intaking = false;

        boolean oldRBumper = false;

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            if (gamepad1.right_bumper && !oldRBumper) {
                robot.extension.runToPreset(Levels.INTAKE);
                robot.arm.runToPreset(Levels.INTAKE);
                robot.claw.startIntake();
                intaking = true;
            }

            if (robot.claw.smartStopDetect(SampleColors.YELLOW, SampleColors.RED) == 1 && intaking) {
                robot.claw.stopIntake();
                robot.arm.runToPreset(Levels.INTERMEDIATE);
                robot.extension.runToPreset(Levels.INTERMEDIATE);
                robot.lift.runToPreset(Levels.INTERMEDIATE);
                intaking = false;
            }

            oldRBumper = gamepad1.right_bumper;

        }
    }
}