package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.*;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import java.lang.Math;

public class MeepMeepAllianceTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);


        Pose2d initialPose = new Pose2d(-31, -61, 0);



        Pose2d initialRungPose = new Pose2d(initialPose.position.x, -35.75 + 12, -Math.PI / 2);

        Pose2d swerveBeamPose = new Pose2d(initialRungPose.position.x + 25.05 + 9, initialRungPose.position.y + 2.875, -Math.PI / 2);
        Pose2d swerveBeamPose2 = new Pose2d(swerveBeamPose.position.x + 2.125 + 5, swerveBeamPose.position.y + 20.5, -Math.PI / 2);
        Pose2d sample1Pose = new Pose2d(swerveBeamPose2.position.x + 11.625, swerveBeamPose2.position.y - 19.375, -Math.PI / 2);




        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(95, 95, 3.2, 3.6, 15.75)
                .setDimensions(15.7,17.2)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(-53,-54,Math.PI/4),Math.PI/9)
                .splineToLinearHeading(new Pose2d(-42, -53, Math.PI/2), Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-49,-62,Math.PI/4),Math.PI/2)
                .splineToLinearHeading(new Pose2d(-52, -52, Math.PI/2), Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-49,-62,Math.PI/4),Math.PI/2)

                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}