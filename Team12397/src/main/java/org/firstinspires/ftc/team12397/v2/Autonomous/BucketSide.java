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

@Autonomous(name="Bucket Side", group="Alliance")
public class BucketSide extends LinearOpMode {
    RobotHardware robot= new RobotHardware(this);
    RoadRunnerActions actionLib= new RoadRunnerActions(robot, this);
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-31, -61, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        Pose2d initialRungPose = new Pose2d(initialPose.position.x, -35.75 + 12, -Math.PI / 2);

        Pose2d swerveBeamPose = new Pose2d(initialRungPose.position.x + 25.05 + 9, initialRungPose.position.y + 2.875, -Math.PI / 2);
        Pose2d swerveBeamPose2 = new Pose2d(swerveBeamPose.position.x + 2.125 + 5, swerveBeamPose.position.y + 20.5, -Math.PI / 2);
        Pose2d sample1Pose = new Pose2d(swerveBeamPose2.position.x + 11.625, swerveBeamPose2.position.y - 19.375, -Math.PI / 2);

        Action ScoreFirst = drive.actionBuilder(initialPose)
                //.setTangent(Math.PI)
                //.lineToX(-53)
                .splineToLinearHeading(new Pose2d(-53,-54,Math.PI/4),Math.PI/9)
                .build();
        Action pickOne= drive.actionBuilder(new Pose2d(-53,-54,Math.PI/4))
                .splineToLinearHeading(new Pose2d(-42, -53, Math.PI/2), Math.PI / 2)

                .build();
        Action scoreOne= drive.actionBuilder(new Pose2d(-42,-53,Math.PI/4))
                .splineToLinearHeading(new Pose2d(-49,-62,Math.PI/4),Math.PI/2)

                .build();
        Action pickTwo= drive.actionBuilder(new Pose2d(-49,-62,Math.PI/4))
                .splineToLinearHeading(new Pose2d(-52, -52, Math.PI/2), Math.PI / 2)

                .build();
        Action scoreTwo= drive.actionBuilder(new Pose2d(-52,-52,Math.PI/4))
                .splineToLinearHeading(new Pose2d(-49,-62,Math.PI/4),Math.PI/2)

                .build();


        robot.init();


        RoadRunnerActions.In in = actionLib.new In();
        RoadRunnerActions.Out out = actionLib.new Out();
        RoadRunnerActions.HorizontalExtender horizontalExtender = actionLib.new HorizontalExtender(hardwareMap);
        RoadRunnerActions.VerticalSlides verticalSlides = actionLib.new VerticalSlides(hardwareMap);

        Actions.runBlocking(new ParallelAction(
        ));

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        out.clawPinch.close(),
                        ScoreFirst,
                        pickOne,
                        scoreOne,
                        pickTwo,
                        scoreTwo
                        //out.takePosition.release(),
                        //new SleepAction(0.6)
                        //out.clawPinch.open(),
                        //out.takePosition.dockOutTake(),

                        //Leg2,
                        //Leg3

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
