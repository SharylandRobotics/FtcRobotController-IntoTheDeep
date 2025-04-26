package org.firstinspires.ftc.team12395.v1.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team12395.v1.RobotHardware;

@Autonomous(name =  "Auto By Encoder 100pts", group = "Robot")
@Disabled

public class AutoByEncoder100 extends LinearOpMode{
    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        robot.init();

        waitForStart();
        runtime.reset();


        //strafing is always 2 inches less than the inches stated in code

        //driving is always 1 inch more than said in code.

        //keeps slides retracted until they are usedpublic class OutTakeHangFirst implements Action {
        //                @Override
        //                public boolean run(@NonNull TelemetryPacket packet) {
        //                    leftOutTake.setPosition(0.67);
        //                    rightOutTake.setPosition(0.33);
        //
        //                    return false;
        //                }
        //            }
        //            public Action outTakeHangFirst(){
        //                return new OutTakeHangFirst();
        //            }
        robot.setExtensionPos(1);
        robot.setOutClawPinch(1);
        robot.setOutTakePos(0);

        robot.driveEncoder(.75,-28.2,-28.2,-28.2,-28.2);
        robot.setSlidePosition(robot.SLIDE_HIGH_RUNG);
        robot.setInClawPitchPos(0);
        sleep(950);
        robot.setSlidePosition(robot.SLIDE_START);
        robot.setOutClawPinch(0);
        robot.setOutTakePos(1);
        sleep(900);
        ///// score preload specimen ^^
        robot.driveEncoder(.5, 17.05,17.05,17.05,17.05);
        robot.driveEncoder(.85, 46.85,46.85,-46.85,-46.85);
        // intake facing submersivle^^
        robot.driveEncoder(.6,44.097,-44.097,-44.097,44.097);
        //robot is current facing leftmost sample
        robot.setInClawPitchPos(1);
        sleep(200);
        robot.setExtensionPos(0);
        sleep(775);
        robot.setInClawPinch(1);
        sleep(300);
        robot.setInClawPitchPos(0);
        robot.setExtensionPos(1);
        sleep(750);
        robot.setOutClawPinch(1);
        sleep(100);
        robot.setInClawPinch(0);
        sleep(500);
        robot.setOutTakePos(2);
        sleep(950);
        robot.setOutClawPinch(0);
        sleep(200);
        robot.setOutTakePos(1);
        robot.driveEncoder(.55,11.25,-11.25,-11.25,11.25);
        robot.setInClawPitchPos(1);
        sleep(200);
        robot.setExtensionPos(0);
        sleep(700);
        robot.setInClawPinch(1);
        sleep(300);
        robot.setExtensionPos(2);
        robot.driveEncoder(.8, -46.5,-46.5,46.5,46.5);
        robot.setInClawPinch(0);
        sleep(200);
        robot.setInClawPitchPos(0);
        robot.setExtensionPos(1);
        sleep(500);
        robot.driveEncoder(.75,54,-54,-54,54);

    }
}
