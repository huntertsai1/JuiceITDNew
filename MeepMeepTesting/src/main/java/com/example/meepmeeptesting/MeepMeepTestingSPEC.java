package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingSPEC {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        Pose2d startPose = new Pose2d(6, -61.8, Math.toRadians(-90));

        double HPDeposit = -51;
        double spikeBack = -16;
        double waits = 0.2;
        double intakeWait = 0.3;

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
                .lineToY(-30,
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim))

                // all spikes

                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(26, -48, Math.toRadians(-90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(36, spikeBack, Math.toRadians(-90)), Math.toRadians(90))

                .splineToLinearHeading(new Pose2d(48, spikeBack, Math.toRadians(-90)), Math.toRadians(-90))

                .setTangent(Math.toRadians(90))
                .lineToY(HPDeposit,
                        new TranslationalVelConstraint(120.0),
                        new ProfileAccelConstraint(-120.0, 120.0))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(44, spikeBack, Math.toRadians(-90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(60, spikeBack, Math.toRadians(-90)), Math.toRadians(-90))

                .setTangent(Math.toRadians(90))
                .lineToY(HPDeposit,
                        new TranslationalVelConstraint(120.0),
                        new ProfileAccelConstraint(-120.0, 120.0))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(49, spikeBack, Math.toRadians(-90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(64, spikeBack, Math.toRadians(-90)), Math.toRadians(-90))

                .setTangent(Math.toRadians(90))
                .lineToY(HPDeposit,
                        new TranslationalVelConstraint(120.0),
                        new ProfileAccelConstraint(-120.0, 120.0))

                // intake spec 2

                .setTangent(Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(21, -46), Math.toRadians(-45))

                // deposit spec 2

                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(3, -31, Math.toRadians(-92)), Math.toRadians(90),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim))

                // intake spec 3

                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(18, -46, Math.toRadians(-45)), Math.toRadians(0),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim))

                // deposit spec 3

                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(0, -31, Math.toRadians(-92)), Math.toRadians(90),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim))

                // intake spec 4

                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(18, -46, Math.toRadians(-45)), Math.toRadians(0),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim))

                // deposit spec 4

                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-3, -31, Math.toRadians(-92)), Math.toRadians(90),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim))

                // intake spec 5

                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(18, -46, Math.toRadians(-45)), Math.toRadians(0),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim))

                // deposit spec 5

                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(0, -31, Math.toRadians(-92)), Math.toRadians(90),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim))

                // park

                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(40, -53, Math.toRadians(-45)), Math.toRadians(0))

                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}