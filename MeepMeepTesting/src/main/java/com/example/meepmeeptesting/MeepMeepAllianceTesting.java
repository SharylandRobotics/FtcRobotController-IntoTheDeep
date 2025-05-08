package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.*;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import java.lang.Math;

public class MeepMeepAllianceTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);


        Pose2d initialPose = new Pose2d(9.5, -61.25, -Math.PI / 2);
        Pose2d SecondPose = new Pose2d(9.5, -38, -Math.PI / 2);
        Pose2d ThirdPose = new Pose2d(59, -55, -Math.PI / 2);
        Pose2d FourthPose = new Pose2d(5, -59, -Math.PI / 2);
        Pose2d FifthPose = new Pose2d(5,-38, -Math.PI /2);
        Pose2d sixthPose = new Pose2d(12,-38, -Math.PI /2);
        Pose2d seventhPose = new Pose2d(55.75,-47.5, -Math.PI /2);



                /*





        Action tab4 = drive.actionBuilder(FourthPose)
                .setTangent(Math.PI / 2)
                .lineToY(FifthPose.position.y)
                .build();

        Action tab5 = drive.actionBuilder(FifthPose)
                .setTangent(0)
                .lineToX(sixthPose.position.x)


                .build();

        Action tab6 = drive.actionBuilder(sixthPose)
                .splineToConstantHeading(new Vector2d(seventhPose.position.x, seventhPose.position.y), 0)
                .build();

        Action tab7 = drive.actionBuilder(seventhPose)


                .setTangent(0)
                .lineToX(4.5)
                .setTangent(Math.PI/2)
                .lineToY(-27)
                .build();

                 */
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(95, 95, 3.2, 3.6, 15.75)
                .setDimensions(15.7,17.2)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(ThirdPose)
                .waitSeconds(0.5)
                .setTangent(0)
                .lineToX(5)
                .setTangent(Math.PI / 2)
                .lineToY(FifthPose.position.y)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}