package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BucketSideTesting {
    public static void main (String []arg){
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(95, 95, 3.2, 3.6, 15.75)
                .setDimensions(15.7,17.2)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-31, -62.3, 0))
                        //.setTangent(Math.PI)
                        .splineToLinearHeading(new Pose2d(-53,-54,Math.PI/4),Math.PI/9)
                //.lineToX(-53)
                .setTangent(Math.PI)
                    .setTangent(0)
                    .splineToLinearHeading(new Pose2d(-53, -53, Math.PI/2), Math.PI / 2)
                        .splineToLinearHeading(new Pose2d(-57,-54,Math.PI/4),Math.PI/2)
                .splineToLinearHeading(new Pose2d(-57, -38, Math.PI/2), Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-57,-54,Math.PI/4),Math.PI/2)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();


    }
}
