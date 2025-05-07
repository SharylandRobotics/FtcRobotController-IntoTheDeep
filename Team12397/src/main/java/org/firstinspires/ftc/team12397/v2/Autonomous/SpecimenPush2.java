package org.firstinspires.ftc.team12397.v2.Autonomous;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team12397.MecanumDrive;
import org.firstinspires.ftc.team12397.v2.RoadRunnerActions;
import org.firstinspires.ftc.team12397.v2.RobotHardware;

import java.lang.Math;

@Autonomous(name="Specimen Push2", group="Alliance")
public class SpecimenPush2 extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    RoadRunnerActions actionLib = new RoadRunnerActions(robot, this);

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(9.5, -61.25, -Math.PI / 2);
        Pose2d SecondPose = new Pose2d(9.5, -38, -Math.PI / 2);
        Pose2d ThirdPose = new Pose2d(59, -55, -Math.PI / 2);
        Pose2d FourthPose = new Pose2d(5, -59, -Math.PI / 2);
        Pose2d FifthPose = new Pose2d(5,-38, -Math.PI /2);
        Pose2d sixthPose = new Pose2d(12,-38, -Math.PI /2);
        Pose2d seventhPose = new Pose2d(55.75,-47.5, -Math.PI /2);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);



        TrajectoryActionBuilder waitHalf = drive.actionBuilder(initialPose)
                .waitSeconds(.5);
        TrajectoryActionBuilder waitOne = drive.actionBuilder(initialPose)
                .waitSeconds(1);


        Action tab1 = drive.actionBuilder(initialPose)
                .lineToY(-38)
                .build();

        Action tab2 = drive.actionBuilder(SecondPose)
                .setTangent(0)
                .lineToX(37)
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(47, -5), 0)
                .setTangent(Math.PI / 2)
                .lineToY(-54)
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(59, -5), 0)
                .setTangent(Math.PI / 2)
                .lineToY(-60)
                .waitSeconds(.75)
                .lineToY(ThirdPose.position.y)
                .build();

        Action tab3 = drive.actionBuilder(ThirdPose)
                .waitSeconds(0.5)
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
                .splineToConstantHeading(new Vector2d(seventhPose.position.x, seventhPose.position.y), 0)
                .build();

        Action tab7 = drive.actionBuilder(seventhPose)


                .setTangent(0)
                .lineToX(4.5)
                .setTangent(Math.PI/2)
                .lineToY(-27)
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
                                out.takePosition.custom(robot.OUTTAKE_ALT),
                                verticalSlides.verticalSlidesToPos(robot.SLIDE_RUNG),
                                out.clawYaw.flipYaw()
                        ),
                        new SleepAction(2),
                        new ParallelAction(
                                verticalSlides.verticalSlidesToPos(robot.SLIDE_ALT),
                                new SequentialAction(
                                        new SleepAction(0.75),
                                        out.clawPinch.open()
                                )
                        ),
                        out.takePosition.custom(robot.OUTTAKE_MID),
                        out.clawYaw.defaultYaw(),
                        new SleepAction(0.5),
                        tab2,
                        tab3,
                        new ParallelAction(
                                tab4
                        ),
                        tab5,
                        new ParallelAction(
                                tab6
                        ),

                        new ParallelAction(
                                tab7
                        ),
                        tab1

                )
        );
    }}