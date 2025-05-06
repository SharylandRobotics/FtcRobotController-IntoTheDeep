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
        Pose2d initialPose = new Pose2d(9.2, -62.3, -Math.PI/2);//12.75
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        Pose2d initialRungPose = new Pose2d(initialPose.position.x, -35.75, -Math.PI/2);

        Pose2d swerveBeamPose = new Pose2d(initialRungPose.position.x + 27.8 , initialRungPose.position.y, -Math.PI/2);
        Pose2d swerveBeamPose2 = new Pose2d(swerveBeamPose.position.x , swerveBeamPose.position.y + 20.5, -Math.PI/2);
        Pose2d sample1Pose = new Pose2d(swerveBeamPose2.position.x + 11.625, swerveBeamPose2.position.y, -Math.PI/2);
        Pose2d sample1Pose2 = new Pose2d(sample1Pose.position.x, sample1Pose.position.y - 14.5, -Math.PI/2);
        Pose2d sample1Pose3 = new Pose2d(sample1Pose2.position.x, swerveBeamPose2.position.y, -Math.PI/2);
        Pose2d sample2Pose = new Pose2d(sample1Pose3.position.x + 6, sample1Pose3.position.y, -Math.PI/2);
        Pose2d sample2Pose2 = new Pose2d(sample2Pose.position.x, sample2Pose.position.y - 14.5, -Math.PI/2);




        Action Leg1 = drive.actionBuilder(initialPose)
                // score FIRST SPECIMEN
                .setTangent(Math.PI/2)
                .lineToYConstantHeading(initialRungPose.position.y)
                .build();

        Action Leg2 = drive.actionBuilder(initialRungPose)
                .setTangent(0)
                .waitSeconds(0)
                // swerve around beam
                .lineToXConstantHeading(swerveBeamPose.position.x)
                .waitSeconds(0)
                // continue swerve ahead of 1st sample
                .setTangent(Math.PI/2)
                .lineToYConstantHeading(swerveBeamPose2.position.y)
                .waitSeconds(0)
                .build();

        Action Leg3 = drive.actionBuilder(swerveBeamPose2)
                // sweep sample into robot
                .setTangent(0)
                .lineToXConstantHeading(sample1Pose.position.x)
                // push sample into zone
                .setTangent(Math.PI/2)
                .lineToYConstantHeading(sample1Pose2.position.y)
                .waitSeconds(0)
                        .build();

        Action Leg4 = drive.actionBuilder(sample1Pose2)
                // sweep sample into robot
                .setTangent(Math.PI/2)
                .lineToYConstantHeading(sample1Pose3.position.y)
                // push sample into zone
                .setTangent(0)
                .lineToXConstantHeading(sample2Pose.position.x)
                .waitSeconds(0)
                .build();

        Action Leg5 = drive.actionBuilder(sample1Pose2)
                // sweep sample into robot
                .setTangent(Math.PI/2)
                .lineToYConstantHeading(sample1Pose3.position.y)
                // push sample into zone
                .setTangent(0)
                .lineToXConstantHeading(sample2Pose.position.x)
                .waitSeconds(0)
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