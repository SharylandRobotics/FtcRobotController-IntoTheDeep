package org.firstinspires.ftc.team12395.v1.auto;


// RR-specific imports
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team12395.v1.MecanumDrive;
import org.firstinspires.ftc.team12395.v1.RobotHardware;

import java.lang.Math;

@Disabled

@Autonomous(name =  "Auto By RoadRunner 100pts", group = "Robot")
public class AutoByRR100 extends LinearOpMode{
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode(){
        robot.init();
        Pose2d initialPose = new Pose2d(9.5, -61.25, -Math.PI/2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .afterTime(0, () -> {
                    robot.setExtensionPos(1);
                    robot.setOutClawPinch(1);
                    robot.setOutTakePos(0);
                    robot.setInClawPitchPos(0);
                })
                .lineToY(-34)
                .stopAndAdd(() ->{
                    robot.setSlidePosition(robot.SLIDE_HIGH_BASKET);
                })
                .waitSeconds(1)
                .stopAndAdd(() -> {
                    robot.setOutClawPinch(0);
                    robot.setOutTakePos(1);
                })

                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(47, -9), 0)
                .setTangent(Math.PI /2)
                .lineToY(-55)
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(56, -11), 0)
                .setTangent(Math.PI / 2)
                .lineToY(-55)
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(64, -13), 0)
                .setTangent(Math.PI / 2)
                .lineToY(-55)
                .stopAndAdd(() -> {
                    robot.setInClawPitchPos(1);
                })
                .waitSeconds(.5)
                .stopAndAdd(() ->{
                    robot.setInClawPinch(1);
                    robot.setExtensionPos(2);

                });
//                .setTangent(Math.PI / 2);
        //.lineToConstantHeading(new Vector2d(0, -32.5));


        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        trajectoryActionChosen = tab1.build();

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        trajectoryActionCloseOut
                )
        );
    }
}