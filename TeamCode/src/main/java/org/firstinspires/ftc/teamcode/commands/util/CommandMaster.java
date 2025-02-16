package org.firstinspires.ftc.teamcode.commands.util;

import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.auton.PreloadEjectFailsafe;
import org.firstinspires.ftc.teamcode.commands.StopIntake;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;

public class CommandMaster {
    Robot robot;

    public CommandMaster(Robot r) {
        robot = r;
    }

    //REGISTER COMMANDS HERE

    public Action stopIntake(SampleColors... colors) {
        return new StopIntake(robot, colors);
    }

    public Action preloadEjectFailSafe() {
        return new PreloadEjectFailsafe(robot);
    }

}
