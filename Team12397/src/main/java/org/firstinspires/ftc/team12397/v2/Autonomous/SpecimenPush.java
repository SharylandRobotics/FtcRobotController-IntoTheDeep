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
    Pose2d initialPose = new Pose2d(9.5, -61.25, -Math.PI / 2);
    MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
    Pose2d latestPose = drive.localizer.getPose();

    private Action updatePose(){
        return (TelemetryPacket packet) -> {
            drive.updatePoseEstimate();
            latestPose = drive.localizer.getPose();
            return false;
        };
    }

    @Override
    public void runOpMode() {

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

        Action Leg1PATH = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(90))
                .lineToY(scorePose.position.y)
                // score
                .build();

        Action drivePATH = drive.actionBuilder(latestPose)
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
                .build();

        Action scoreBreakPATH = drive.actionBuilder(latestPose)
                .setTangent(Math.toRadians(150))
                .splineToConstantHeading(scorePose2.position, Math.toRadians(beamAngle))
                        .build();

        Action scoreBreakFinalPATH = drive.actionBuilder(latestPose)
                .setTangent(Math.toRadians(-55))
                .splineToConstantHeading(retrievePose.position, Math.toRadians(-70))
                .build();

        Action scorePATH = drive.actionBuilder(latestPose)
                // start at retrieval here
                .setTangent(Math.toRadians(110))
                .splineToConstantHeading(scorePose2.position, Math.toRadians(beamAngle))
                .build();


        Action scoreBackPATH = drive.actionBuilder(latestPose)
                // start at scoring
                .setTangent(Math.toRadians(-55))
                .splineToConstantHeading(retrievePose.position, Math.toRadians(-70))
                .build();

        /*
         * if calling finishScore immediately after, wait for 0.2 to allow servos to flip.
         * Adjust correctly to pathing times.
         */
        Action prepScore = new ParallelAction(
                out.takePosition.custom(robot.OUTTAKE_ALT),
                verticalSlides.verticalSlidesToPos(robot.SLIDE_RUNG),
                out.clawYaw.flipYaw()
        );

        /*
         * recommend to wait for 0.2 sec after calling for claw to open
         */
        Action finishScore = new ParallelAction(
                verticalSlides.verticalSlidesToPos(robot.SLIDE_ALT),
                new SequentialAction(
                        new SleepAction(0.5),
                        out.clawPinch.open()
                )
        );

        Action awaitRetrieval = new ParallelAction(
                out.takePosition.custom(robot.OUTTAKE_MID),
                out.clawYaw.defaultYaw(),
                verticalSlides.verticalSlidesToPos(0)
        );



        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        // score 1 & pushing
                        new ParallelAction(
                                Leg1PATH,
                                prepScore
                        ),
                        new SleepAction(0.2),
                        finishScore,
                        new SleepAction(0.2),
                        new ParallelAction(
                                awaitRetrieval,
                                new SequentialAction(
                                        updatePose(), // UPDATE POSE
                                        new SleepAction(0.5),
                                        drivePATH
                                )
                        ),

                        // pickup 1
                        out.clawPinch.close(),
                        new SleepAction(0.5),
                        updatePose(), // UPDATE POSE

                        new ParallelAction(
                                verticalSlides.verticalSlidesToPos(8),
                                scoreBreakPATH,
                                new SequentialAction(
                                        new SleepAction(1),
                                        prepScore
                                )
                        ),
                        finishScore,
                        updatePose(), // UPDATE POSE
                        new SleepAction(0.2),
                        // score 2

                        new ParallelAction(
                                scoreBreakFinalPATH,
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        awaitRetrieval
                                )
                        ),
                        out.clawPinch.close(),
                        updatePose(), // UPDATE POSE
                        new SleepAction(0.5),
                        // pickup 2


                        new ParallelAction(
                                prepScore,
                                scorePATH
                        ),
                        finishScore,
                        updatePose(), // UPDATE POSE
                        new SleepAction(0.2),
                        // score 3


                        new ParallelAction(
                                awaitRetrieval,
                                scoreBackPATH
                        ),
                        out.clawPinch.close(),
                        updatePose(), // UPDATE POSE
                        new SleepAction(0.5),
                        // pickup 3


                        new ParallelAction(
                                prepScore,
                                scorePATH
                        ),
                        finishScore,
                        updatePose(), // UPDATE POSE
                        new SleepAction(0.2),
                        // score 4


                        new ParallelAction(
                                awaitRetrieval,
                                scoreBackPATH
                        ),
                        out.clawPinch.close(),
                        updatePose(), // UPDATE POSE
                        new SleepAction(0.5),
                        // pickup 4


                        new ParallelAction(
                                prepScore,
                                scorePATH
                        ),
                        finishScore,
                        updatePose(), // UPDATE POSE
                        new SleepAction(0.2),
                        // score 5


                        new ParallelAction(
                                awaitRetrieval,
                                scoreBackPATH
                        ),
                        updatePose(), // UPDATE POSE
                        new SleepAction(1)
                        // park END
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