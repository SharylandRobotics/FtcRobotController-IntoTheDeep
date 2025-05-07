package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.*;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.lang.Math;

public class MeepMeepAllianceTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        Pose2d initialPose = new Pose2d(9.2, -62.3, -Math.PI/2);//12.75


        Pose2d initialRungPose = new Pose2d(initialPose.position.x, -40, -Math.PI/2);

        Pose2d swerveBeamPose = new Pose2d(initialRungPose.position.x + 29 , initialRungPose.position.y, -Math.PI/2);
        Pose2d swerveBeamPose2 = new Pose2d(swerveBeamPose.position.x , swerveBeamPose.position.y + 28.5, -Math.PI/2);
        Pose2d sample1Pose = new Pose2d(swerveBeamPose2.position.x + 11.625, swerveBeamPose2.position.y, -Math.PI/2);
        Pose2d sample1Pose2 = new Pose2d(sample1Pose.position.x, sample1Pose.position.y - 36, -Math.PI/2);
        Pose2d sample1Pose3 = new Pose2d(sample1Pose2.position.x, swerveBeamPose2.position.y, -Math.PI/2);
        Pose2d sample2Pose = new Pose2d(sample1Pose3.position.x + 6, sample1Pose3.position.y, -Math.PI/2);
        Pose2d sample2Pose2 = new Pose2d(sample2Pose.position.x, sample1Pose.position.y - 36, -Math.PI/2);

        Pose2d dropOff = new Pose2d(initialRungPose.position.x - 3, initialRungPose.position.y, -Math.PI/2);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(95, 95, 3.2, 3.6, 15.75)
                .setDimensions(15.7,17.2)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(sample2Pose)

                // score FIRST SPECIMEN
                .setTangent(Math.PI/2)
                .waitSeconds(0)
                .lineToYConstantHeading(sample2Pose.position.y)
                // push sample into zone
                .setTangent(6*Math.PI/7)
                .waitSeconds(1)
                .lineToXConstantHeading(dropOff.position.x)
                .waitSeconds(0)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}