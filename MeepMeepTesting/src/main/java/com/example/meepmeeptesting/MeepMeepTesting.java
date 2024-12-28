package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(6, -62, Math.toRadians(-90));

        double HPDeposit = -46;
        double spikeBack = -13;
        double waits = 0.2;
        
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(12.5, 17.5)
                .setConstraints(72, 72, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(startPose)

                //PRELOAD
                .lineToY(-33)
                .waitSeconds(waits)

                //ALL SPIKES
                .splineToLinearHeading(new Pose2d(30, -38, Math.toRadians(-90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(40, -12, Math.toRadians(-90)), Math.toRadians(90))
                .setTangent(Math.toRadians(0))
                .lineToX(44)
                .setTangent(Math.toRadians(90))
                .lineToY(HPDeposit)
                .lineToY(-12)
                .setTangent(Math.toRadians(0))
                .lineToX(54)
                .setTangent(Math.toRadians(90))
                .lineToY(HPDeposit)
                .lineToY(-12)
                .setTangent(Math.toRadians(0))
                .lineToX(64)
                .setTangent(Math.toRadians(90))
                .lineToY(HPDeposit)

                //DEPOSITS
                .splineToLinearHeading(new Pose2d(16, -44, Math.toRadians(-45)), Math.toRadians(180))
                .waitSeconds(waits)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(6, -33, Math.toRadians(-90)), Math.toRadians(90))
                .waitSeconds(waits)



//                .splineToLinearHeading(new Pose2d(24, -44, Math.toRadians(-45)), Math.toRadians(180))
//                .waitSeconds(waits)
//                .setTangent(Math.toRadians(180))
//                .splineTo(new Vector2d(4, -33), Math.toRadians(135))
//                .waitSeconds(waits)
//                .splineToLinearHeading(new Pose2d(24, -44, Math.toRadians(-45)), Math.toRadians(180))
//                .waitSeconds(waits)
//                .setTangent(Math.toRadians(180))
//                .splineTo(new Vector2d(4, -33), Math.toRadians(135))
//                .waitSeconds(waits)
//                .splineToLinearHeading(new Pose2d(24, -44, Math.toRadians(-45)), Math.toRadians(180))
//                .waitSeconds(waits)
//                .setTangent(Math.toRadians(180))
//                .splineTo(new Vector2d(4, -33), Math.toRadians(135))
//                .waitSeconds(waits)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}