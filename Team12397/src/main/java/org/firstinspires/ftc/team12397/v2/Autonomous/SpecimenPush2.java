package org.firstinspires.ftc.team12397.v2.Autonomous;

import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team12397.MecanumDrive;
import org.firstinspires.ftc.team12397.v2.RoadRunnerActions;
import org.firstinspires.ftc.team12397.v2.RobotHardware;
import org.opencv.core.Mat;

import java.lang.Math;

@Autonomous(name="Specimen Push2", group="Alliance")
public class SpecimenPush2 extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    RoadRunnerActions actionLib = new RoadRunnerActions(robot, this);

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(9.5, -61.25, -Math.PI / 2);
        Pose2d SecondPose = new Pose2d(9.5, -37.9, -Math.PI / 2);
        Pose2d ThirdPose = new Pose2d(61, -62.2, -Math.PI / 2);
        Pose2d FourthPose = new Pose2d(5, -55, -Math.PI / 2);
        Pose2d FifthPose = new Pose2d(5,-39.3, -Math.PI /2);
        Pose2d sixthPose = new Pose2d(12,-39.3, -Math.PI /2);
        Pose2d seventhPose = new Pose2d(53,-62.2, -Math.PI /2);

        Pose2d eigthPose = new Pose2d(10, -39.3, -Math.PI/2);
        Pose2d ninthPose = (seventhPose);
        Pose2d tenthPose = new Pose2d(8, -39.3, - Math.PI/2);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);



        TrajectoryActionBuilder waitHalf = drive.actionBuilder(initialPose)
                .waitSeconds(.5);
        TrajectoryActionBuilder waitOne = drive.actionBuilder(initialPose)
                .waitSeconds(1);


        Action tab1 = drive.actionBuilder(initialPose)
                .setTangent(Math.PI/2)
                .lineToY(SecondPose.position.y)
                .build();

        Action tab2 = drive.actionBuilder(SecondPose)
                .setTangent(0)
                .lineToX(37)
                .setTangent(Math.PI/2)
                .splineToConstantHeading(new Vector2d(48, -10), 0)
                .setTangent(Math.PI / 2)
                .lineToY(-54)
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(61, -12), 0)
                .setTangent(Math.PI / 2)
                .lineToY(ThirdPose.position.y)
                //

                .build();

        Action tab3 = drive.actionBuilder(ThirdPose)
                .lineToY(-55)
                .waitSeconds(0)
                .setTangent(0)
                .lineToX(5)
                .build();

        Action tab4 = drive.actionBuilder(FourthPose)
                .setTangent(Math.PI / 2)
                .lineToY(FifthPose.position.y)
                .build();

        Action tab5 = drive.actionBuilder(FifthPose)
                .setTangent(0)
                .lineToX(sixthPose.position.x)


                .build();

        Action tab6 = drive.actionBuilder(sixthPose)
                .splineToConstantHeading(new Vector2d(seventhPose.position.x, seventhPose.position.y), -90)
                .build();

        Action tab7 = drive.actionBuilder(seventhPose)


                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(5, sixthPose.position.y), Math.toRadians(90))
                .build();

        Action tab8 = drive.actionBuilder(eigthPose)
                        .setTangent(Math.toRadians(-45))
                                .splineToConstantHeading(new Vector2d(seventhPose.position.x, seventhPose.position.y), Math.toRadians(-90))
                                        .build();

        Action tab9 = drive.actionBuilder(ninthPose)
                        .setTangent(0)
                                .splineToConstantHeading(new Vector2d(4, tenthPose.position.y), Math.toRadians(90))
                                        .build();

        Action tab91 = drive.actionBuilder(new Pose2d(4, tenthPose.position.y, -Math.PI/2))
                .lineToX(8)
                .build();

        Action tab10 = drive.actionBuilder(tenthPose)
                        .setTangent(Math.toRadians(-45))
                                .splineToConstantHeading(new Vector2d(seventhPose.position.x, seventhPose.position.y), Math.toRadians(-90))
                                        .build();

        robot.init();

        RoadRunnerActions.In in = actionLib.new In();
        RoadRunnerActions.Out out = actionLib.new Out();
        RoadRunnerActions.HorizontalExtender horizontalExtender = actionLib.new HorizontalExtender(hardwareMap);
        RoadRunnerActions.VerticalSlides verticalSlides = actionLib.new VerticalSlides(hardwareMap);

        Actions.runBlocking(new ParallelAction(
                in.clawPinch.open(),
                in.clawPitch.rotatePitch(-1),
                in.clawYaw.rotateYaw(0),

                out.clawPinch.close(),
                out.clawYaw.defaultYaw(),
                out.takePosition.custom(robot.OUTTAKE_MID-0.1),

                verticalSlides.verticalSlidesToPos(0),
                horizontalExtender.extenderToPos(0)
        ));

        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                tab1,
                                new SequentialAction(

                                ),
                                out.takePosition.custom(robot.OUTTAKE_ALT),
                                verticalSlides.verticalSlidesToPos(robot.SLIDE_RUNG),
                                out.clawYaw.flipYaw()
                        ),
                        new SleepAction(0.2),
                        new ParallelAction(
                                verticalSlides.verticalSlidesToPos(robot.SLIDE_ALT),
                                new SequentialAction(
                                        new SleepAction(0.4),
                                        out.clawPinch.open()
                                )
                        ),
                        new SleepAction(0.2),
                        new ParallelAction(
                                out.takePosition.custom(robot.OUTTAKE_MID),
                                tab2,
                                new SequentialAction(
                                        new SleepAction(0.2),
                                        out.clawYaw.defaultYaw(),
                                        verticalSlides.verticalSlidesToPos(0)
                                )
                        ),
                        out.clawPinch.close(),
                        new SleepAction(0.6),
                        verticalSlides.verticalSlidesToPos(8),

                        tab3,
                        new ParallelAction(
                                tab4,
                                out.takePosition.custom(robot.OUTTAKE_ALT),
                                verticalSlides.verticalSlidesToPos(robot.SLIDE_RUNG),
                                out.clawYaw.flipYaw()
                        ),
                        new ParallelAction(
                                verticalSlides.verticalSlidesToPos(robot.SLIDE_ALT),
                                new SequentialAction(
                                        new SleepAction(0.7),
                                        tab5,
                                        new SleepAction(0.5),
                                        out.clawPinch.open()
                                )
                        ),
                        new SleepAction(0.2),
                        new ParallelAction(
                                out.takePosition.custom(robot.OUTTAKE_MID),
                                tab6,
                                new SequentialAction(
                                        new SleepAction(0.2),
                                        out.clawYaw.defaultYaw(),
                                        verticalSlides.verticalSlidesToPos(0)
                                )
                        ),
                        out.clawPinch.close(),
                        new SleepAction(0.6),
                        new ParallelAction(
                                verticalSlides.verticalSlidesToPos(robot.SLIDE_RUNG),
                                tab7,
                                new SequentialAction(
                                        new SleepAction(1),
                                        out.takePosition.custom(robot.OUTTAKE_ALT)
                                )
                        ),
                        new ParallelAction(
                                verticalSlides.verticalSlidesToPos(robot.SLIDE_ALT),
                                new SequentialAction(
                                        new SleepAction(1),
                                        tab5,
                                        new SleepAction(0.5),
                                        out.clawPinch.open()
                                )
                        ),
                        out.takePosition.custom(robot.OUTTAKE_MID),
                        tab8,
                        out.clawPinch.close(),
                        new SleepAction(0.5),
                        new ParallelAction(
                            verticalSlides.verticalSlidesToPos(robot.SLIDE_RUNG),
                            tab9,
                            new SequentialAction(
                                new SleepAction(1),
                                out.takePosition.custom(robot.OUTTAKE_ALT)
                            )
                        ),
                        new ParallelAction(
                            verticalSlides.verticalSlidesToPos(robot.SLIDE_ALT),
                            new SequentialAction(
                                    new SleepAction(1.5),
                                    tab91,
                                    new SleepAction(0.5),
                                    out.clawPinch.open()
                            )
                        ),
                        tab10

                )
        );
    }}