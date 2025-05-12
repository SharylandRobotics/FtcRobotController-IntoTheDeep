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
        Pose2d initialPose = new Pose2d(9.2, -62.3, 0);//12.75
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        Pose2d initialRungPose = new Pose2d(initialPose.position.x, -35.75+12, -Math.PI/2);

        Pose2d swerveBeamPose = new Pose2d(initialRungPose.position.x + 25.05 + 9, initialRungPose.position.y+2.875, -Math.PI/2);
        Pose2d swerveBeamPose2 = new Pose2d(swerveBeamPose.position.x + 2.125 + 5, swerveBeamPose.position.y+20.5, -Math.PI/2);
        Pose2d sample1Pose = new Pose2d(swerveBeamPose2.position.x + 11.625, swerveBeamPose2.position.y-19.375, -Math.PI/2);

        // this autonomous is meant to score a preload specimen, push 3 samples, and score as many specimen
        // as possible. Currently, the path goes to the rungs, then splines ahead of the 1st sample and tries to push it.
        // Attempt to finish this by trial and error as the roadrunner is not as accurate as meant to be.
        // replace anything as you see fit as long as you make progress :)

        Action Leg1 = drive.actionBuilder(initialPose)
                // score FIRST SPECIMEN
                .setTangent(Math.PI/2)
                .lineToYLinearHeading(initialRungPose.position.y, -Math.PI/2)
                .build();

        Action Leg2 = drive.actionBuilder(initialRungPose)
                .setTangent(-Math.PI /8)
                .waitSeconds(0)
                // swerve around beam
                .splineToConstantHeading(swerveBeamPose.position, -Math.PI/2)
                .waitSeconds(0)
                // continue swerve ahead of 1st sample
                .setTangent(Math.PI/3)
                .splineToConstantHeading(swerveBeamPose2.position, -Math.PI/2)
                .waitSeconds(0)
                .build();

        Action Leg3 = drive.actionBuilder(sample1Pose)
                // sweep sample into robot
                .setTangent(Math.PI/4)
                .splineToConstantHeading(sample1Pose.position, -Math.PI/2)
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
                out.takePosition.release(),

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