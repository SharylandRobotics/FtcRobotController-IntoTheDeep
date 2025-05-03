package org.firstinspires.ftc.team00000.v3.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team00000.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team00000.v3.RobotHardware;
import org.firstinspires.ftc.team00000.v3.RobotState;

import java.lang.Math;

@Config
@Autonomous(name = "Specimen Cyclic", group = "Autonomous")
public class SpecimenCyclic extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode(){

        double ROBOT_WIDTH = 14.75;
        double ROBOT_HEIGHT = 16;
        double CENTER_X = ROBOT_WIDTH / 2;
        double CENTER_Y = ROBOT_HEIGHT / 2;
        double FIELD = 70.5;

        Pose2d startPose = new Pose2d(CENTER_X,-FIELD + CENTER_Y, Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        robot.init();

        // -------------------- Preloaded Specimen Score --------------------
        Action preloadScore = drive.actionBuilder(startPose)
                .lineToY(-FIELD + 48 - CENTER_Y)
                .build();

        Action clearSub = drive.actionBuilder(new Pose2d(CENTER_X, -FIELD + 48 - CENTER_Y, Math.PI/2))
                .setTangent(-Math.PI / 2)
                .lineToY(-36)
                .splineTo(new Vector2d(45, -CENTER_Y), Math.PI / 2)
                .build();

        // -------------------- Sample Push Cycles --------------------
        Action pushSamples = drive.actionBuilder(new Pose2d(45, - CENTER_Y, -Math.PI / 2))
                .setTangent(2 * Math.PI)
                .splineToConstantHeading(new Vector2d(48, -54), -Math.PI / 2)
                .waitSeconds(0)
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(56, -CENTER_Y), 2 * Math.PI)
                .build();

        Action intakeWall_1 = drive.actionBuilder(new Pose2d(56, - CENTER_Y, -Math.PI / 2))
                .setTangent(2 * Math.PI)
                .splineToConstantHeading(new Vector2d(58, -67), -Math.PI / 2)
                .build();

        Actions.runBlocking(robot.movePivot(robot.PIVOT_SCORE));
        Actions.runBlocking(robot.moveArm(robot.ARM_SCORE));

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Status", "Waiting for Start");
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;
        Actions.runBlocking (new SequentialAction(

                // Score preloaded specimen
                new ParallelAction(
                        packet -> {
                            RobotState.applyState(robot, RobotState.State.HIGH_SCORE);
                            return false;
                        },
                        preloadScore
                ),
                robot.moveClaw(robot.CLAW_OPEN),
                sleepAction(300),

                new ParallelAction(
                        robot.moveSlide(robot.SLIDE_MINIMUM),
                        clearSub
                ),

                new ParallelAction(
                        packet -> {
                            RobotState.applyState(robot, RobotState.State.STOWED);
                            return false;
                            },
                        pushSamples
                ),

                new ParallelAction(
                        packet -> {
                            RobotState.applyState(robot, RobotState.State.WALL_INTAKE);
                            return false;
                        },
                        intakeWall_1
                )
        ));
    }

    private Action sleepAction(long millis) {
        return packet -> {
            sleep(millis);
            return false;
        };
    }
}