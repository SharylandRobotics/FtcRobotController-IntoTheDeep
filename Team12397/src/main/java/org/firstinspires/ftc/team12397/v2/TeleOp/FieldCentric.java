package org.firstinspires.ftc.team12397.v2.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team12397.v2.RobotHardware;


@TeleOp(name="Robot Centric (Duo)", group="Robot")

public class FieldCentric extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        double drive = 0;
        double strafe = 0;
        double turn = 0;

        double slide = 0;

        double secondLeg = 0.5;
        double inClawPitch = 0;
        double inClawPinch = 0;
        double inClawYaw = 0;
        double outClawPinch = 0;
        double outClawYaw = 0;
        double extendPosition = 0;


        int pinchTimer = 0;
        boolean pinchToggle = false;

        int outPinchTimer = 0;
        boolean outPinchToggle = false;
        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn  =  gamepad1.right_stick_x;

            // Combine drive and turn for blended motion. Use RobotHardware class
            robot.driveFieldCentric(drive, strafe, turn);

            // Use gamepad left Bumper to open and close the intake claw

            // toggle pinch logic
            if (gamepad2.left_bumper){
                if (pinchTimer >= 4) {
                    pinchToggle = !pinchToggle;
                    pinchTimer = 0;
                }
            }
            if(pinchToggle) {
                inClawPinch = 1;
            }else{
                inClawPinch = 0;
            }
            robot.setInClawPinch(inClawPinch);



            // THESE NUMBERS ARE NOT SERVO POSITIONS! look in RobotHardware.setInClawYaw() for positions
            // run the nudge(r) process if claw isn't closed and claw isn't backwards and extender isn't back
            if ( (inClawPinch == 0 && pinchTimer > 4) && (inClawPitch == 1 && extendPosition > 0.4) ) {
                // > 0.05 to prevent unprompted movement (trigger drift)
                if (gamepad2.left_trigger > 0.05) {
                    inClawYaw += 0.2 * gamepad2.left_trigger;
                    inClawYaw = Math.min(1, inClawYaw);
                }
                if (gamepad2.right_trigger > 0.05) {
                    inClawYaw -= 0.2 * gamepad2.right_trigger;
                    inClawYaw = Math.max(0, inClawYaw);
                }
            } else {
                // if the claw is closed, wait for 5 thread loops (250 milis) and reset inClawYaw.
                inClawYaw = 0;
            }
            robot.setInClawYaw(inClawYaw);


            //Use right bumper to open and close outtake claw
            if (gamepad2.right_bumper){
                if (outPinchTimer >= 4) {
                    outPinchToggle = !outPinchToggle;
                    outPinchTimer = 0;
                }
            }
            if(outPinchToggle){
                outClawPinch = 1;
            }else{
                outClawPinch = 0;
            }
            robot.setOutClawPinch(outClawPinch);


            // Move both secondLeg servos to new position.  Use RobotHardware class
            // Use gamepad buttons to move arm up (Y) and down (A)
            if (gamepad2.y) {
                secondLeg = 1;
                outClawYaw = 1;
                // switch to a less steep angle and turn around claw
            } else if (gamepad2.b) {
                secondLeg = 0;
                outClawYaw = 0;
                inClawPitch = -1;
                // servo to transfer tuck in pitch yaw to 0
            }


            //moves vertical slides
            if(gamepad1.dpad_up) {
                slide = 114;
            }if (gamepad1.dpad_left){
                slide = robot.SLIDE_HIGH_BASKET;
            }

            if (-gamepad2.right_stick_y == 1){
                slide = robot.SLIDE_HIGH_BASKET;
            } else if (-gamepad2.right_stick_y == -1){
                slide = 0;
            } else if (gamepad2.right_stick_x == 1){
                slide = 114;
            } else if (gamepad2.right_stick_x == -1){
                slide = robot.SLIDE_HANG_RUNG;
            }


            if (gamepad2.dpad_right) {
                extendPosition = 0.4;
                inClawPitch = 0;
                secondLeg = 0.5;
                inClawYaw = 0;
            } else if (gamepad2.dpad_up) {
                extendPosition = 1;
                inClawPitch = 1;
                pinchToggle = false;
                inClawYaw = 0;

            } else if(gamepad2.dpad_down){
                extendPosition = 0;
                inClawPitch = 0;
                secondLeg = 0.5;
            }else if (Math.round(Math.abs(gamepad2.left_stick_y*20)) != 0){
                // if alex moves his left stick up/down more than 2.5% of maximum movement...
                /* throttle extendPosition : prevents it from being a huge/small #
                 which the player would have to decrease manually
                 not after setting the value because setExtensionPos already handles the parameter
                 & you'd still decrease the previous #
                */
                extendPosition = Math.min(1, extendPosition);
                extendPosition = Math.max(0, extendPosition);
                // set target position to previous distance +/- fudge amount
                extendPosition = extendPosition + -gamepad2.left_stick_y/5;
                if (-gamepad2.left_stick_y == -1){
                    extendPosition = 0;
                }

            }

            robot.setOutClawYaw(outClawYaw);
            robot.setOutTakePos(secondLeg);
            robot.setSlidePosition(slide);
            robot.setInClawPitchPos(inClawPitch);
            robot.setExtensionPos(extendPosition);

            // Send telemetry messages to explain controls and show robot status
            telemetry.addData("Drive", "Left Stick");
            telemetry.addData("Turn", "Right Stick");
            telemetry.addData("Slide Up/Down", "Dpad_Up & Dpad_Down");
            telemetry.addData("HSlide Up/Down", "Dpad_Right & Dpad_Left");
            telemetry.addData("Vertical", "Y & A Buttons");
            telemetry.addData("Horizontal", "B & X Buttons");
            telemetry.addData("-", "-------");

            telemetry.addData("Drive Power", "%.2f", drive);
            telemetry.addData("Turn Power",  "%.2f", turn);
            telemetry.addData("Slide Power",  "%.2f", slide);
            telemetry.addData("Extend %", "%.2f", extendPosition);
            telemetry.addData("Vertical Power", "%.2f", secondLeg);
            telemetry.addData("Pitch Position", "%.2f", inClawPitch);
            telemetry.addData("Hand Position",  "Offset = %.2f", extendPosition);
            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            pinchTimer++;
            outPinchTimer++;
            sleep(50);
        }
    }
}
