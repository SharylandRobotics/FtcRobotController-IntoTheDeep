package org.firstinspires.ftc.team12397.v2.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team12397.v2.RobotHardware;


@TeleOp(name="Debug OpMode", group="Robot")

public class DebugOp extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        double slide = 0;
        double secondLeg = 0.5;
        double inClawPitch = 0;
        double extendPosition = 0;
        double inClawPinch = 0;
        double outClawPinch = 0;
        double inClawYaw = 0;

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // interpreted drive, 0.4 closed, 0 open
            robot.setInClawPinch(inClawPinch);


            robot.setInClawYaw(inClawYaw);


            // interpreted drive, 1 closed, 0.7 open
            robot.setOutClawPinch(outClawPinch);



            robot.setOutTakePos(secondLeg);

            robot.setSlidePosition(slide*robot.TICKS_PER_INCH);

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
