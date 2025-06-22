package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingSAMP {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        Pose2d startPose = new Pose2d(-38, -62, Math.toRadians(90));

        double depositX = -52.8;
        double depositY = -48.5;
        double depositWait = 1.2;
        double intakeWait = 0.9;

        double veloLim = 60.0;
        double accelUpperLim = 60.0;
        double accelLowerLim = -40.0;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(12.5, 17.5)
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 12)
                .build();


        myBot.runAction(myBot.getDrive().actionBuilder(startPose)

                // preload

                .setTangent(Math.toRadians(160))
                .splineToLinearHeading(new Pose2d(depositX, depositY, Math.toRadians(60)), Math.toRadians(90))
                .waitSeconds(depositWait)

                // intake 1

                .lineToY(-42, new TranslationalVelConstraint(20))

                // deposit 1

                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-60, depositY, Math.toRadians(85)), Math.toRadians(135))
                .setTangent(Math.toRadians(85))

                // intake 2

                .lineToY(-42, new TranslationalVelConstraint(20))
                .waitSeconds(intakeWait + depositWait)

                // deposit 2

                .setTangent(Math.toRadians(150))
                .splineToLinearHeading(new Pose2d(-62, -48, Math.toRadians(100)), Math.toRadians(150))
                .setTangent(Math.toRadians(100))

                // intake 3

                .lineToY(-40, new TranslationalVelConstraint(20))
                .waitSeconds(intakeWait)

                // deposit 3

                .setTangent(Math.toRadians(272))
                .splineToLinearHeading(new Pose2d(-62, depositY, Math.toRadians(90)), Math.toRadians(272))
                .waitSeconds(depositWait)

                // sub drive

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-48, -11, Math.toRadians(0)), Math.toRadians(0))
                .lineToX(-20)

                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}