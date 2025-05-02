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

@Autonomous(name="Specimen Push", group="Alliance")
public class SpecimenPush extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    RoadRunnerActions actionLib = new RoadRunnerActions(robot, this);

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(10, -61.875, Math.PI / 2 );
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);



        Action Leg1 = drive.actionBuilder(initialPose)
                // score FIRST SPECIMEN
                .lineToY(-34)
                .build();

        Action Leg2 = drive.actionBuilder(new Pose2d(initialPose.position.x, -34, Math.PI /2))
                .setTangent(-Math.PI /8)
                .waitSeconds(0)
                // swerve around beam
                .splineToSplineHeading(new Pose2d(new Vector2d(34.25, -31.125), 4*Math.PI/5), Math.PI/3,
                        new TranslationalVelConstraint(80), new ProfileAccelConstraint(-75, 100))
                .waitSeconds(0)
                // continue swerve ahead of 1st sample
                .setTangent(Math.PI/3)
                .splineToLinearHeading(new Pose2d(new Vector2d(36.375, -10.625), 4*Math.PI/3), Math.PI/4,
                        new TranslationalVelConstraint(80), new ProfileAccelConstraint(-75, 100))
                .waitSeconds(0)
                .build();

        Action Leg3 = drive.actionBuilder(new Pose2d(36.375, -10.625, 4*Math.PI/3))
                // sweep sample into robot
                .setTangent(Math.PI/4)
                .splineToSplineHeading(new Pose2d(new Vector2d(48, -30), -Math.PI/2), 3*Math.PI/2)
                .waitSeconds(0)
                // push sample into zone
                .lineToYConstantHeading(-47)
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
                out.takePosition.dockOutTake(),

                verticalSlides.verticalSlidesToPos(0),
                horizontalExtender.extenderToPos(0)
                ));

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                    new ParallelAction(
                            Leg1,
                            new SequentialAction(
                                    //new SleepAction(1),
                                    //out.takePosition.prime()
                            )
                    ),
                    //out.takePosition.release(),
                    new SleepAction(0.6),
                    //out.clawPinch.open(),
                    //out.takePosition.dockOutTake(),

                    Leg2,
                    Leg3

                )
        );

    }
    private Action sleepAction(long milliseconds) {
        return (TelemetryPacket packet) -> {
            sleep(milliseconds);
            return false;
        };
    }
}