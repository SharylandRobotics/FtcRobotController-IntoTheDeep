package org.firstinspires.ftc.team12397.v2.Autonomous;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team12397.MecanumDrive;
import org.firstinspires.ftc.team12397.v2.RoadRunnerActions;
import org.firstinspires.ftc.team12397.v2.RobotHardware;

import java.lang.Math;

@Disabled
@Autonomous(name="Specimen Push", group="Alliance")
public class SpecimenPush extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    RoadRunnerActions actionLib = new RoadRunnerActions(robot, this);

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(9.2, -62.3, -Math.PI/2);//12.75
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        Pose2d initialRungPose = new Pose2d(initialPose.position.x, -40, -Math.PI/2);

        Pose2d swerveBeamPose = new Pose2d(initialRungPose.position.x + 29 , initialRungPose.position.y, -Math.PI/2);
        Pose2d swerveBeamPose2 = new Pose2d(swerveBeamPose.position.x , swerveBeamPose.position.y + 28.5, -Math.PI/2);
        Pose2d sample1Pose = new Pose2d(swerveBeamPose2.position.x + 11.625, swerveBeamPose2.position.y, -Math.PI/2);
        Pose2d sample1Pose2 = new Pose2d(sample1Pose.position.x, sample1Pose.position.y - 38, -Math.PI/2);
        Pose2d sample1Pose3 = new Pose2d(sample1Pose2.position.x, swerveBeamPose2.position.y, -Math.PI/2);
        Pose2d sample2Pose = new Pose2d(sample1Pose3.position.x + 7, sample1Pose3.position.y, -Math.PI/2);
        Pose2d sample2Pose2 = new Pose2d(sample2Pose.position.x, sample1Pose.position.y - 37, -Math.PI/2);


        Pose2d sample3Pose = new Pose2d(sample1Pose3.position.x + 5, sample1Pose3.position.y, -Math.PI/2);
        Pose2d sample3Pose2 = new Pose2d(sample2Pose.position.x, sample1Pose.position.y - 37, -Math.PI/2);

        Pose2d dropOff = new Pose2d(initialRungPose.position.x - 3, initialRungPose.position.y, -Math.PI/2);

        Action Leg1 = drive.actionBuilder(initialPose)
                // score FIRST SPECIMEN
                .setTangent(Math.PI/2)
                .lineToYConstantHeading(initialRungPose.position.y)
                .build();

        Action Leg2 = drive.actionBuilder(initialRungPose)
                .setTangent(0)
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
                .waitSeconds(0)
                // push sample into zone
                .setTangent(Math.PI/2)
                .lineToYConstantHeading(sample1Pose2.position.y)
                .waitSeconds(0)
                        .build();

        Action Leg4 = drive.actionBuilder(sample1Pose2)
                // sweep sample into robot
                .setTangent(Math.PI/2)
                .lineToYConstantHeading(sample1Pose3.position.y)
                .waitSeconds(0)
                // push sample into zone
                .setTangent(0)
                .lineToXConstantHeading(sample2Pose.position.x)
                .waitSeconds(0)
                .build();

        Action Leg5 = drive.actionBuilder(sample2Pose)
                // push sample into zone
                .setTangent(Math.PI/2)
                .waitSeconds(0)
                .lineToYConstantHeading(sample2Pose2.position.y)
                .waitSeconds(0)
                .build();


        Action Drop1 = drive.actionBuilder(sample2Pose2)
                // push sample into zone
                .setTangent(Math.toRadians(160))
                .waitSeconds(0)
                .lineToXConstantHeading(dropOff.position.x)
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
                out.takePosition.custom(robot.OUTTAKE_MID-0.1),

                verticalSlides.verticalSlidesToPos(0),
                horizontalExtender.extenderToPos(0)
                ));

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        showEstimate(drive),
                    new ParallelAction(
                            Leg1,
                            new SequentialAction(
                                    new SleepAction(1),
                                    new ParallelAction(
                                            //out.takePosition.custom(robot.OUTTAKE_ALT),
                                            //verticalSlides.verticalSlidesToPos(robot.SLIDE_RUNG)
                                    )
                            )
                    ),
                    showEstimate(drive),
                    new SleepAction(0.6),
                    //verticalSlides.verticalSlidesToPos(robot.SLIDE_ALT),
                    new SleepAction(1),
                    //out.clawPinch.open(),
                    new SleepAction(1),
                    new ParallelAction(
                            //out.takePosition.custom(robot.OUTTAKE_MID),
                            //verticalSlides.verticalSlidesToPos(0)

                            ),
                    showEstimate(drive),
                    Leg2,
                    showEstimate(drive),
                    Leg3,
                    showEstimate(drive),
                    Leg4,
                    showEstimate(drive),
                    Leg5,
                    showEstimate(drive),
                    Drop1,
                        showEstimate(drive),
                        new SleepAction(10)

                )
        );

    }
    private Action sleepAction(long milliseconds) {
        return (TelemetryPacket packet) -> {
            sleep(milliseconds);
            return false;
        };
    }

    private Action showEstimate(MecanumDrive d){
        return (TelemetryPacket packet) -> {
            telemetry.addData(d.updatePoseEstimate().toString(), "");
            telemetry.update();
            return false;
        };
    }
}