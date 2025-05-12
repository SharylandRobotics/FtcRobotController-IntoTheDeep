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
        Pose2d ThirdPose = new Pose2d(41, -62.2, -Math.PI / 2);
        Pose2d FourthPose = new Pose2d(5, -55, -Math.PI / 2);
        Pose2d FifthPose = new Pose2d(5,-39.3, -Math.PI /2);
        Pose2d sixthPose = new Pose2d(10,-39.3, -Math.PI /2);
        Pose2d seventhPose = new Pose2d(41,-62.2, -Math.PI /2);

        Pose2d eigthPose = new Pose2d(7, -39.3, -Math.PI/2);
        Pose2d ninthPose = (seventhPose);
        Pose2d tenthPose = new Pose2d(0, -39.3, - Math.PI/2);

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
                .splineToConstantHeading(new Vector2d(45, -8), 0)
                .splineToConstantHeading(new Vector2d(53, -10), Math.toRadians(-90))
                .lineToY(-54)
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(55, -13), 0)
                .splineToConstantHeading(new Vector2d(65, -16), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(68, seventhPose.position.y+10), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(60, seventhPose.position.y+14), Math.toRadians(165))
                .splineToConstantHeading(new Vector2d(seventhPose.position.x, seventhPose.position.y), Math.toRadians(-90))

                //

                .build();


        Action tab4 = drive.actionBuilder(ThirdPose)
                .setTangent(Math.toRadians(160))
                .splineToConstantHeading(new Vector2d(3, sixthPose.position.y), Math.toRadians(120)) // -1.5
                .build();


        Action tab6 = drive.actionBuilder(sixthPose)
                .splineToConstantHeading(new Vector2d(seventhPose.position.x, seventhPose.position.y), Math.toRadians(-90))
                .build();

        Action tab7 = drive.actionBuilder(seventhPose)


                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(eigthPose.position.x, sixthPose.position.y), Math.toRadians(90))
                .build();

        Action tab72 = drive.actionBuilder(seventhPose)


                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(eigthPose.position.x-14, sixthPose.position.y), Math.toRadians(90))
                .build();

        Action tab8 = drive.actionBuilder(eigthPose)
                        .setTangent(Math.toRadians(-45))
                                .splineToConstantHeading(new Vector2d(seventhPose.position.x, seventhPose.position.y+6), Math.toRadians(-90))
                .lineToYConstantHeading(seventhPose.position.y)
                                        .build();

        Action tab9 = drive.actionBuilder(ninthPose)
                        .setTangent(Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(0, tenthPose.position.y), Math.toRadians(90))
                                        .build();



        Action tab10 = drive.actionBuilder(tenthPose)
                        .setTangent(Math.toRadians(-45))
                                .splineToConstantHeading(new Vector2d(seventhPose.position.x, seventhPose.position.y), Math.toRadians(-90))
                                        .build();

        Action tab11 = drive.actionBuilder(seventhPose)
                        .setTangent(Math.toRadians(-45))
                                .splineToLinearHeading(new Pose2d(seventhPose.position.x+3, seventhPose.position.y-3, Math.toRadians(-45)), Math.toRadians(-45)
                                , new TranslationalVelConstraint(120))
                                        .build();

        robot.init(false);

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
                                        new SleepAction(0.7),
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
                                out.clawYaw.flipYaw(),
                                new SequentialAction(
                                        new SleepAction(1),
                                        out.takePosition.custom(robot.OUTTAKE_ALT)
                                )
                        ),
                        new ParallelAction(
                                verticalSlides.verticalSlidesToPos(robot.SLIDE_ALT),
                                new SequentialAction(
                                        new SleepAction(0.7),
                                        out.clawPinch.open()
                                )
                        ),
                        out.takePosition.custom(robot.OUTTAKE_MID),
                        new ParallelAction(
                                tab8,
                                verticalSlides.verticalSlidesToPos(0),
                                out.clawYaw.defaultYaw()
                                // here
                        ),
                        out.clawPinch.close(),
                        new SleepAction(0.5),
                        new ParallelAction(
                            verticalSlides.verticalSlidesToPos(robot.SLIDE_RUNG),
                            tab9,
                            out.clawYaw.flipYaw(),
                            new SequentialAction(
                                new SleepAction(1),
                                out.takePosition.custom(robot.OUTTAKE_ALT)
                            )
                        ),
                        new ParallelAction(
                            verticalSlides.verticalSlidesToPos(robot.SLIDE_ALT),
                            new SequentialAction(
                                    new SleepAction(0.7),
                                    out.clawPinch.open()
                            )
                        ),
                        new SleepAction(0.2),
                        new ParallelAction(
                                out.takePosition.custom(robot.OUTTAKE_MID),
                                tab10,
                                new SequentialAction(
                                        out.clawYaw.defaultYaw(),
                                        verticalSlides.verticalSlidesToPos(0)
                                )
                        ),
                        out.clawPinch.close(),
                        new SleepAction(0.6),
                        new ParallelAction(
                                verticalSlides.verticalSlidesToPos(robot.SLIDE_RUNG),
                                tab72,
                                out.clawYaw.flipYaw(),
                                new SequentialAction(
                                        new SleepAction(1),
                                        out.takePosition.custom(robot.OUTTAKE_ALT)
                                )
                        ),
                        new ParallelAction(
                                verticalSlides.verticalSlidesToPos(0),
                                new SequentialAction(
                                        new SleepAction(0.7),
                                        out.clawPinch.open()
                                )
                        ),
                        new SleepAction(0.5),
                        new ParallelAction(
                                tab11,
                                horizontalExtender.extenderToPos(1),
                                in.clawPitch.rotatePitch(robot.PITCH_MID)
                        )


                )
        );
    }}