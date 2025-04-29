package org.firstinspires.ftc.team12397.v2.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team12397.v2.RobotHardware;


@TeleOp(name="Debug OpMode", group="Robot")
@Config
public class DebugOp extends LinearOpMode {

    public static double slide = 0;
    public static double secondLeg = 0.5;
    public static double inClawPitch = 0;
    public static double extendPosition = 0;
    public static double inClawPinch = 0;
    public static double outClawPinch = 0;
    public static double inClawYaw = 0;
    public static double outClawYaw = 0;
    public static boolean singleServo = false;
    public static double leftOut = 0;
    public static double rightOut = 0;

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();



        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // interpreted drive, 0.35 closed, 0 open
            robot.setInClawPinch(inClawPinch);


            robot.setInClawYaw(inClawYaw);


            // interpreted drive, 1 closed, 0.7 open
            robot.setOutClawPinch(outClawPinch);

            robot.setOutClawYaw(outClawYaw);



            robot.setSlidePosition(slide*robot.TICKS_PER_INCH);

            if (singleServo){
                robot.leftOutTake.setPosition(leftOut);
                robot.rightOutTake.setPosition(rightOut);
            } else {
                robot.setOutTakePos(secondLeg);
            }
            // 1 down, 0.25 angled
            robot.setInClawPitchPos(inClawPitch);

            robot.setExtensionPos(extendPosition);

            telemetry.addData("Slide Power",  "%.2f", slide);
            telemetry.addData("Extend %", "%.2f", extendPosition);
            telemetry.addData("Vertical Power", "%.2f", secondLeg);
            telemetry.addData("Pitch Position", "%.2f", inClawPitch);
            telemetry.addData("Hand Position",  "Offset = %.2f", extendPosition);
            telemetry.update();

            sleep(50);
        }
    }
}
