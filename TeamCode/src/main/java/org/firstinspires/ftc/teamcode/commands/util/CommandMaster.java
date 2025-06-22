package org.firstinspires.ftc.teamcode.commands.util;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.AutoCenterSpecimen;
import org.firstinspires.ftc.teamcode.commands.auton.PreloadEjectFailsafe;
import org.firstinspires.ftc.teamcode.commands.StopIntake;
import org.firstinspires.ftc.teamcode.commands.auton.StopIntakeTimeout;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;

public class CommandMaster {
    Robot robot;

    public CommandMaster(Robot r) {
        robot = r;
    }

    //REGISTER COMMANDS HERE

    public Action stopIntake(Gamepad gamepad, SampleColors... colors) {
        return new StopIntake(robot, gamepad, colors);
    }

    public Action stopIntake(SampleColors... colors) {
        return new StopIntake(robot, colors);
    }

    public Action stopIntakeTimeout(long timeout, SampleColors... colors) {
        return new StopIntakeTimeout(robot, timeout, colors);
    }

    public Action autoCenterSpecimen() {
        return new AutoCenterSpecimen(robot);
    }

    public Action preloadEjectFailSafe() {
        return new PreloadEjectFailsafe(robot);
    }

}
