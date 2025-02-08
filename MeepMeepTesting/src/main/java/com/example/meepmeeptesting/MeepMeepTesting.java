package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(6, -62, Math.toRadians(-90));

        double HPDeposit = -53;
        double spikeBack = -12;
        double waits = 0.2;

        double veloLim = 60.0;
        double accelUpperLim = 60.0;
        double accelLowerLim = -40.0;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(12.5, 17.5)
                .setConstraints(84, 84, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-38, -62, Math.toRadians(0)))
                .setTangent(Math.toRadians(160))
                .splineToLinearHeading(new Pose2d(-53.5, -53.5, Math.toRadians(45)), Math.toRadians(160))
                .waitSeconds(1.2)
                .build()
        );

//        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
//
//                //PRELOAD
//                .lineToY(-30,
//                        new TranslationalVelConstraint(veloLim),
//                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim))
//
//                //ALL SPIKES
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(22, -42, Math.toRadians(-90)), Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(36, spikeBack, Math.toRadians(-90)), Math.toRadians(90))
//
//                .splineToLinearHeading(new Pose2d(46, spikeBack, Math.toRadians(-90)), Math.toRadians(-90))
//
////                .setTangent(Math.toRadians(90))
////                .lineToY(HPDeposit)
//                .splineToLinearHeading(new Pose2d(46, HPDeposit, Math.toRadians(-90)), Math.toRadians(-90))
//
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(46, spikeBack, Math.toRadians(-90)), Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(55, spikeBack, Math.toRadians(-90)), Math.toRadians(-90))
//
//                .setTangent(Math.toRadians(90))
//                .lineToY(HPDeposit)
//
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(55, spikeBack, Math.toRadians(-90)), Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(61, spikeBack, Math.toRadians(-90)), Math.toRadians(-90))
//
//                .setTangent(Math.toRadians(90))
//                .lineToY(HPDeposit)
//
////                .setTangent(Math.toRadians(0))
////                .lineToX(46)
////                .setTangent(Math.toRadians(90))
////                .lineToY(HPDeposit)
////                .lineToY(spikeBack)
////                .setTangent(Math.toRadians(0))
////                .lineToX(55)
////                .setTangent(Math.toRadians(90))
////                .lineToY(HPDeposit)
////                .lineToY(spikeBack)
////                .setTangent(Math.toRadians(0))
////                .lineToX(61.5)
////                .setTangent(Math.toRadians(90))
////                .lineToY(HPDeposit)
//
//                //DEPOSITS
//
//                // 2ND SPEC
//                .setTangent(Math.toRadians(-90))
//                .strafeToLinearHeading(new Vector2d(19, -48), Math.toRadians(-45))
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(3, -30, Math.toRadians(-92)), Math.toRadians(90),
//                        new TranslationalVelConstraint(veloLim),
//                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim))
//
////                //3RD SPEC
////                .setTangent(Math.toRadians(-90))
////                .splineToLinearHeading(new Pose2d(17, -46, Math.toRadians(-45)), Math.toRadians(0),
////                        new TranslationalVelConstraint(veloLim),
////                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim))
////                .waitSeconds(1)
////                .setTangent(Math.toRadians(180))
////                .splineToLinearHeading(new Pose2d(6, -33, Math.toRadians(-92)), Math.toRadians(90),
////                        new TranslationalVelConstraint(veloLim),
////                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim))
////                .waitSeconds(waits)
//
////                //4TH SPEC
////
////                .splineTo(new Vector2d(16, -44), Math.toRadians(-45))
////                .waitSeconds(1)
////                .setTangent(Math.toRadians(180))
////                .splineToLinearHeading(new Pose2d(6, -31, Math.toRadians(-97)), Math.toRadians(90))
////                .waitSeconds(waits)
////
////                //5TH SPEC
////
////                .splineTo(new Vector2d(16, -44), Math.toRadians(-45))
////                .waitSeconds(1)
////                .setTangent(Math.toRadians(180))
////                .splineToLinearHeading(new Pose2d(6, -31, Math.toRadians(-97)), Math.toRadians(90))
////                .waitSeconds(waits)
////                .splineTo(new Vector2d(16, -44), Math.toRadians(-45))
//
//                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}