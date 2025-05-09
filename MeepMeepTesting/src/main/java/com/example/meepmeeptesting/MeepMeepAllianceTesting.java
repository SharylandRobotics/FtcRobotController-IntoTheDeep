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
        Pose2d scorePose = new Pose2d(9.5, -37.9, -Math.PI / 2);
        Pose2d beamPose1 = new Pose2d(32, -36, -Math.PI / 2);
        Pose2d beamPose2 = new Pose2d(40, -18, -Math.PI / 2);
        Pose2d beamCurve = new Pose2d(48,-19, -Math.PI /2);
        Pose2d pushEndPose1 = new Pose2d(48,-50, -Math.PI /2);

        Pose2d pushStartPose = new Pose2d(53, -24, -Math.PI/2);
        Pose2d pushCurvePose = new Pose2d(54, -16.5, -Math.PI / 2);
        Pose2d pushCurvePose2 = new Pose2d(58, -18, -Math.PI/2);
        Pose2d pushEndPose2 = new Pose2d(58,-50, -Math.PI /2);

        Pose2d pushStartPoseW = new Pose2d(58, -22, -Math.PI/2);
        Pose2d pushCurvePoseW = new Pose2d(59, -14, -Math.PI / 2);
        Pose2d pushCurvePose2W = new Pose2d(62, -13, -Math.PI/2);
        Pose2d pushEndPose2W = new Pose2d(62,-48, -Math.PI /2);

        Pose2d mateCurve = new Pose2d(58, -61.25, -Math.PI/2);

        Pose2d scorePose2 = new Pose2d(9.5, -37.9, -Math.PI/2);
        Pose2d retrievePose = new Pose2d(36, -61.25, -Math.PI/2);

        double beamAngle = 130;



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

        Pose2d eigthPose = new Pose2d(10, -39.3, -Math.PI/2);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(95, 95, 3.2, 3.6, 15.75)
                .setDimensions(15.7,17.2)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .setTangent(Math.toRadians(90))
                .lineToY(scorePose.position.y)
                // score

                // start action 1
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(beamPose1.position, Math.toRadians(30))
                .splineToConstantHeading(beamPose2.position, Math.toRadians(30))
                .splineToConstantHeading(beamCurve.position, Math.toRadians(-90))
                .lineToYConstantHeading(pushEndPose1.position.y)

                .lineToYConstantHeading(pushStartPose.position.y)
                .splineToConstantHeading(pushCurvePose.position, Math.toRadians(0))
                .splineToConstantHeading(pushCurvePose2.position, Math.toRadians(-90))
                .lineToYConstantHeading(pushEndPose2.position.y)

                .lineToYConstantHeading(pushStartPoseW.position.y)
                .splineToConstantHeading(pushCurvePoseW.position, Math.toRadians(45))
                .splineToConstantHeading(pushCurvePose2W.position, Math.toRadians(-90))
                .lineToYConstantHeading(pushEndPose2W.position.y)

                .splineToConstantHeading(mateCurve.position, Math.toRadians(-90))
                // end action 1

                // start score break 1
                .setTangent(Math.toRadians(150))
                .splineToConstantHeading(scorePose2.position, Math.toRadians(beamAngle))
                // break 2nd scoring here
                        .setTangent(Math.toRadians(-55))
                .splineToConstantHeading(retrievePose.position, Math.toRadians(-70))

                // score loop
                .setTangent(Math.toRadians(110))
                .splineToConstantHeading(scorePose2.position, Math.toRadians(beamAngle))
                // break scoring here
                .setTangent(Math.toRadians(-55))
                .splineToConstantHeading(retrievePose.position, Math.toRadians(-70))
                // end score ACT, retrieve, repeat

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}