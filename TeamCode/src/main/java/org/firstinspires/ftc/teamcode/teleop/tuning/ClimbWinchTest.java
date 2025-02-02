package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.WinchAlign;
import org.firstinspires.ftc.teamcode.commands.WinchStopAction;
import org.firstinspires.ftc.teamcode.commands.WinchTimeAction;
import org.firstinspires.ftc.teamcode.subsystems.lift.Lift;
import org.firstinspires.ftc.teamcode.util.hardware.ContinuousServo;
import org.firstinspires.ftc.teamcode.util.hardware.Motor;
import org.firstinspires.ftc.teamcode.util.hardware.StepperServo;

import java.util.ArrayList;
import java.util.List;

@TeleOp(group = "tuning")
@Config
//@Disabled
public class ClimbWinchTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, false);
        //robot.climbWinch.equalServosStart();
        List<Action> actionsQueue = new ArrayList<>();

        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();

            if (gamepad1.dpad_up) {
                actionsQueue.add(new WinchStopAction(robot.climbWinch, 10000, telemetry));
            }
            if (gamepad1.dpad_down) {
                actionsQueue.add(new WinchStopAction(robot.climbWinch, 500, telemetry));
            }
            if (gamepad1.circle){
                actionsQueue.add(new WinchAlign(robot.climbWinch, telemetry));
            }

            List<Action> newActions = new ArrayList<>();
            for (Action action : actionsQueue) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            actionsQueue = newActions;
            telemetry.addData("dpad", gamepad1.dpad_up);
            telemetry.update();
            robot.climbWinch.update();
        }
    }
}


// INTAKE (DOWN)
// Arm: 103, elbow: 282, ext 190
// INTAKE (INTERMEDIATE)
// Arm: 130, elbow 282, ext 190
// INTERMEDIATE
// Arm: 260, Elbow: 190, Ext: 100

// LIFT MAX 2150
