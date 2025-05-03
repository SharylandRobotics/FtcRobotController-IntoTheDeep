package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepMecanumTesting_00000 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double ROBOT_WIDTH = 14.75;
        double ROBOT_HEIGHT = 16;
        double CENTER_X = ROBOT_WIDTH / 2;
        double CENTER_Y = ROBOT_HEIGHT / 2;
        double FIELD = 70.5;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(95, 95, 3.2, 3.6, 13.25)
                .setDimensions(ROBOT_WIDTH,ROBOT_HEIGHT)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(CENTER_X, -FIELD + CENTER_Y, Math.PI / 2 ))

                // score FIRST SPECIMEN
                .lineToY(-FIELD + 48 - CENTER_Y)

                .setTangent(-Math.PI / 2)
                .lineToY(-36)
                .splineTo(new Vector2d(46, -CENTER_Y), Math.PI / 2)

                .setTangent(2 * Math.PI)
                .splineToConstantHeading(new Vector2d(48, -48), -Math.PI / 2)

                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(56, -CENTER_Y), 2 * Math.PI)

                .setTangent(2 * Math.PI)
                .splineToConstantHeading(new Vector2d(58, -48), -Math.PI / 2)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}