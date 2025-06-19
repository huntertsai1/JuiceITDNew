package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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

        double HPDeposit = -51;
        double spikeBack = -16;
        double waits = 0.2;
        double intakeWait = 0.3;

        double veloLim = 50.0;
        double accelUpperLim = 50;
        double accelLowerLim = -30.0;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(12.5, 17.5)
                .setConstraints(120, 120, Math.toRadians(180), Math.toRadians(180), 12)
                .build();


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(6, -61.8, Math.toRadians(-90)))
                // preload
                .lineToY(-28,
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim))

                //spikes
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(22, -42, Math.toRadians(-90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(36, spikeBack, Math.toRadians(-90)), Math.toRadians(90))

                .splineToLinearHeading(new Pose2d(46, spikeBack, Math.toRadians(-90)), Math.toRadians(-90))

                .setTangent(Math.toRadians(90))
                .lineToY(HPDeposit,
                        new TranslationalVelConstraint(120.0),
                        new ProfileAccelConstraint(-120.0, 120.0))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(46, spikeBack, Math.toRadians(-90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(55, spikeBack, Math.toRadians(-90)), Math.toRadians(-90))

                .setTangent(Math.toRadians(90))
                .lineToY(HPDeposit,
                        new TranslationalVelConstraint(120.0),
                        new ProfileAccelConstraint(-120.0, 120.0))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(55, spikeBack, Math.toRadians(-90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(61.5, spikeBack, Math.toRadians(-90)), Math.toRadians(-90))

                .setTangent(Math.toRadians(90))
                .lineToY(HPDeposit,
                        new TranslationalVelConstraint(120.0),
                        new ProfileAccelConstraint(-120.0, 120.0))

                .setTangent(Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(19, -48), Math.toRadians(-45))

                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(3, -30, Math.toRadians(-92)), Math.toRadians(90),
                        new TranslationalVelConstraint(veloLim),
                        new ProfileAccelConstraint(accelLowerLim, accelUpperLim))



//                // Automated cycling
//
//                    .setTangent(Math.toRadians(-90))
//                    .splineToLinearHeading(new Pose2d(20, -47, Math.toRadians(-45)), Math.toRadians(0),
//                            new TranslationalVelConstraint(veloLim),
//                            new ProfileAccelConstraint(accelLowerLim, accelUpperLim))
//
//                        .setTangent(Math.toRadians(180))
//                        .splineToLinearHeading(new Pose2d(depoTargetX, -29, Math.toRadians(-92)), Math.toRadians(90),
//                                new TranslationalVelConstraint(veloLim),
//                                new ProfileAccelConstraint(accelLowerLim, accelUpperLim))

//                //preload
//                .setTangent(Math.toRadians(160))
//                .splineToLinearHeading(new Pose2d(depositX, depositY, Math.toRadians(45)), Math.toRadians(160))
//                .waitSeconds(0.6)
//
//                //spike1
//                .setTangent(Math.toRadians(45))
//                .splineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(106)), Math.toRadians(45))
//                .waitSeconds(0.5)
//
//                //depo1
//                .setTangent(Math.toRadians(225))
//                .splineToLinearHeading(new Pose2d(depositX, depositY, Math.toRadians(30)), Math.toRadians(200))
//                .waitSeconds(1.2)
//
//                //spike2
//                .setTangent(Math.toRadians(92))
//                .splineToLinearHeading(new Pose2d(-58, -48, Math.toRadians(112)), Math.toRadians(100))
//                .waitSeconds(0.5)
//
//                //depo2
//                .setTangent(Math.toRadians(272))
//                .splineToLinearHeading(new Pose2d(depositX, depositY, Math.toRadians(35)), Math.toRadians(272))
//                .waitSeconds(1.2)
//
//                //spike3
//                .setTangent(Math.toRadians(88))
//                .splineToLinearHeading(new Pose2d(-63, -47, Math.toRadians(110)), Math.toRadians(95))
//                .waitSeconds(0.5)
//
//                //depo3
//                .setTangent(Math.toRadians(268))
//                .splineToLinearHeading(new Pose2d(depositX, depositY, Math.toRadians(30)), Math.toRadians(268))
//                .waitSeconds(1.2)
//
//                //ascent zone park
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(-48, -11, Math.toRadians(0)), Math.toRadians(0))
//                .lineToX(-25)


                .build()

        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}