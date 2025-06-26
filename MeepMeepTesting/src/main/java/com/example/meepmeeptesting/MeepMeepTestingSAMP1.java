package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingSAMP1 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        Pose2d startPose = new Pose2d(-38, -62, Math.toRadians(0));

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
                .setTangent(Math.toRadians(160))
                .splineToLinearHeading(new Pose2d(depositX, depositY, Math.toRadians(45)), Math.toRadians(160))
                .waitSeconds(1)

                //spike1
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-45, -49, Math.toRadians(106)), Math.toRadians(45))
                .waitSeconds(intakeWait)
                .lineToY(-43,
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim))

                //depo1
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(depositX, depositY, Math.toRadians(30)), Math.toRadians(200))
                .waitSeconds(1)

                //spike2
                .setTangent(Math.toRadians(92))
                .splineToLinearHeading(new Pose2d(-58, -48, Math.toRadians(112)), Math.toRadians(100))
                .waitSeconds(1)
                .setTangent(Math.toRadians(112))
                .lineToY(-43,
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim))


                //depo2
                .setTangent(Math.toRadians(272))
                .splineToLinearHeading(new Pose2d(depositX, depositY, Math.toRadians(40)), Math.toRadians(272))
                .waitSeconds(1)

                //spike3
                .setTangent(Math.toRadians(88))
                .splineToLinearHeading(new Pose2d(-64.5, -45, Math.toRadians(119)), Math.toRadians(95))
                .waitSeconds(1)
                .setTangent(Math.toRadians(119))
                .lineToY(-43,
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim))

                //depo3
                .setTangent(Math.toRadians(268))
                .splineToLinearHeading(new Pose2d(depositX, depositY, Math.toRadians(40)), Math.toRadians(268))
                .waitSeconds(1)

                //ascent zone park
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-48, -11, Math.toRadians(0)), Math.toRadians(0))
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