package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingSAMP2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        Pose2d startPose = new Pose2d(-38, -62, Math.toRadians(90));

        double depositX = -53;
        double depositY = -49;
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

                //preload
                .setTangent(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-54, -52), Math.toRadians(72))

                .setTangent(Math.toRadians(72))
                .lineToY(-54)

                .waitSeconds(1)


                //spike1
                .setTangent(Math.toRadians(72))
                .lineToY(-48,
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim))

                //depo1
                .setTangent(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-58, -52), Math.toRadians(90))
                .waitSeconds(1)

                //spike2
                .setTangent(Math.toRadians(90))
                .lineToY(-48,
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim))

                //depo2
                .setTangent(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-56, -52), Math.toRadians(60))
                .waitSeconds(1)

                // spike 3

                .setTangent(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-64, -50), Math.toRadians(105))
                .waitSeconds(1)

                .setTangent(Math.toRadians(105))
                .lineToY(-46,
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim))

                // depo3

                .setTangent(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-56, -52), Math.toRadians(60))
                .waitSeconds(1)

                // spike run

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-20, -11, Math.toRadians(0)), Math.toRadians(0))

                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}