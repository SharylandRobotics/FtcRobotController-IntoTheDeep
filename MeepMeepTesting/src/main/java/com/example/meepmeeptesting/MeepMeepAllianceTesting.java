package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepAllianceTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(95, 95, 3.2, 3.6, 15.75)
                .setDimensions(15.75,17.25)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -61.875, Math.PI / 2 ))

                // score FIRST SPECIMEN
                .lineToY(-34)
                .setTangent(-Math.PI /8)

                // swerve around beam
                .splineToSplineHeading(new Pose2d(new Vector2d(34.25, -31.125), 4*Math.PI/5), Math.PI/3,
                        new TranslationalVelConstraint(80), new ProfileAccelConstraint(-75, 100))

                // continue swerve ahead of 1st sample
                .setTangent(Math.PI/3)
                .splineToLinearHeading(new Pose2d(new Vector2d(36.375, -10.625), 4*Math.PI/3), Math.PI/4,
                        new TranslationalVelConstraint(80), new ProfileAccelConstraint(-75, 100))

                // sweep sample into robot
                .setTangent(Math.PI/4)
                .splineToSplineHeading(new Pose2d(new Vector2d(48, -30), -Math.PI/2), 3*Math.PI/2)

                // push sample into zone
                .lineToYConstantHeading(-47)

                // swerve ahead of 2nd sample
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(new Vector2d(52, -16), Math.PI*3.2/2), -Math.PI/4)

                // sweep sample into robot
                .setTangent(-Math.PI/4)
                .splineToSplineHeading(new Pose2d(new Vector2d(58, -32), -Math.PI/2), 3.025*Math.PI/2)

                // push sample into zone
                .setTangent(3.025*Math.PI/2)
                .lineToY(-47)

                // drive ahead of sample
                .setTangent(Math.PI/2)
                .splineToConstantHeading(new Vector2d(58, -14), Math.PI/2,
                        new TranslationalVelConstraint(80), new ProfileAccelConstraint(-75, 100))

                // swerve to meet sample
                .setTangent(Math.PI/2)
                .splineToConstantHeading(new Vector2d(62.5, -15), -Math.PI/2)

                // push sample into zone
                .setTangent(-Math.PI/2)
                .lineToY(-47, new TranslationalVelConstraint(80), new ProfileAccelConstraint(-75, 100))

                // pick up specimen
                .waitSeconds(1)

                // drive to rungs
                .setTangent(Math.toRadians(175))
                .lineToXLinearHeading(10, Math.toRadians(90))

                // scoring loop
                                .setTangent(Math.toRadians(-15))
                                .lineToYLinearHeading(-52, Math.toRadians(-90))

                                .setTangent(Math.toRadians(-15))
                                .lineToXLinearHeading(10, Math.toRadians(90))

                // scoring loop 2
                .setTangent(Math.toRadians(-15))
                .lineToYLinearHeading(-52, Math.toRadians(-90))

                .setTangent(Math.toRadians(-15))
                .lineToXLinearHeading(10, Math.toRadians(90))


                // drive into zone
                .setTangent(Math.toRadians(-15))
                .lineToY(-52)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}