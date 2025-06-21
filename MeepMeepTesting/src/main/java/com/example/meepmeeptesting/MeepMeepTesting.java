package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        Pose2d startPose = new Pose2d(6, -62, Math.toRadians(-90));

        double depositX = -52.8;
        double depositY = -48.5;
        double depositWait = 1.2;
        double intakeWait = 0.9;

        double subSampleX = -6;
        double subSampleY = -12;

        double veloLim = 50.0;
        double accelUpperLim = 50;
        double accelLowerLim = -30.0;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(12.5, 17.5)
                .setConstraints(120, 120, Math.toRadians(180), Math.toRadians(180), 12)
                .build();


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-40, -62, Math.toRadians(90)))
                // preload = drive.actionBuilder(startPose)
                        //preload
                        .setTangent(Math.toRadians(160))
                        .splineToLinearHeading(new Pose2d(depositX, depositY, Math.toRadians(60)), Math.toRadians(90))
                        .waitSeconds(depositWait)
             //spike1 = preload.endTrajectory().fresh()
                        //spike1
                                .lineToY(-42, new TranslationalVelConstraint(20))
//
//        // deposit1 = spike1.endTrajectory().fresh()
//                //depo1
//                .setTangent(Math.toRadians(268))
//                .splineToLinearHeading(new Pose2d(depositX, depositY, Math.toRadians(60)), Math.toRadians(268))
//                .waitSeconds(depositWait)
        // spike2 = deposit1.endTrajectory().fresh()
                //spike2
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-60, depositY, Math.toRadians(85)), Math.toRadians(135))
                .setTangent(Math.toRadians(85))
                .lineToY(-42, new TranslationalVelConstraint(20))
                .waitSeconds(intakeWait + depositWait)


        // spike3 = deposit2.endTrajectory().fresh()
                //spike3
                .setTangent(Math.toRadians(150))
                .splineToLinearHeading(new Pose2d(-62, -48, Math.toRadians(100)), Math.toRadians(150))
                .setTangent(Math.toRadians(100))
                .lineToY(-40, new TranslationalVelConstraint(20))
                .waitSeconds(intakeWait)


//        // deposit3 = spike3.endTrajectory().fresh()
//                //depo3
                .setTangent(Math.toRadians(272))
                .splineToLinearHeading(new Pose2d(-62, depositY, Math.toRadians(90)), Math.toRadians(272))
                .waitSeconds(depositWait)



//        // intake4 = deposit3.endTrajectory().fresh()
//                // sub intake
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(subSampleX-20, subSampleY, Math.toRadians(0)), Math.toRadians(0))
//
//        // deposit4 = intake4.endTrajectory().fresh()
//                // depo 4
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(depositX, depositY, Math.toRadians(40)), Math.toRadians(268))
//
//        // subDrive = deposit3.endTrajectory().fresh() // CHANGE TO DEPOSIT4 TO DRIVE BACK TO PARK AFTER SUB INTAKE + DEPO
//                //ascent zone park
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(-48, -11, Math.toRadians(0)), Math.toRadians(0))
//                .lineToX(-20)
                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}