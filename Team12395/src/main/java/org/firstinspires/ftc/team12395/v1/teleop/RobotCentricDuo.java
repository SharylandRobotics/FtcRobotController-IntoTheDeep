package org.firstinspires.ftc.team12395.v1.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team12395.v1.RobotHardware;


@TeleOp(name="Robot Centric (Duo)", group="Robot")

public class RobotCentricDuo extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        double drive = 0;
        double strafe = 0;
        double turn = 0;
        double slide = 0;
        double hslide = 0;
        double secondLeg = 0.5;
        double inClawPitch = 0;
        double extendOffset = 1;
        double inClawPinch = 0;
        double outClawPinch = 0;
        double rotation = 0;

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
                if (pinchTimer >= 10) {
                    pinchToggle = !pinchToggle;
                    pinchTimer = 0;
                }
            }
            if(pinchToggle) {
                inClawPinch = 1;
            }else{
                inClawPinch = 0;
            }
            robot.setInClawPosition(inClawPinch);



            // THESE NUMBERS ARE NOT SERVO POSITIONS! look in RobotHardware.setInClawRotation() for positions
            // run the nudge(r) process if claw isn't closed
            if (inClawPinch == 0 && pinchTimer < 5) {
                // > 0.05 to prevent unprompted movement (trigger drift)
                if (gamepad2.left_trigger > 0.05) {
                    rotation += 0.05 * gamepad2.left_trigger;
                    rotation = Math.min(1, rotation);
                }
                if (gamepad2.right_trigger > 0.05) {
                    rotation -= 0.05 * gamepad2.right_trigger;
                    rotation = Math.max(-0.82775, rotation);
                }
            } else {
                // if the claw is closed, wait for 5 thread loops (250 milis) and reset rotation.
                rotation = 0;
            }
            robot.setInClawRotation(rotation);


            //Use right bumper to open and close outtake claw
            if (gamepad2.right_bumper){
                if (outPinchTimer >= 10) {
                    outPinchToggle = !outPinchToggle;
                    outPinchTimer = 0;
                }
            }
            if(outPinchToggle){
                outClawPinch = 1;
            }else{
                outClawPinch = 0;
            }
            robot.setOutClawPosition(outClawPinch);


            // Move both secondLeg servos to new position.  Use RobotHardware class
            // Use gamepad buttons to move arm up (Y) and down (A)
            if (gamepad2.a) {
                secondLeg = 1;
            } else if (gamepad2.y) {
                secondLeg = 0;
            } else if (gamepad2.b) {
                secondLeg = 3;
            }


            //moves vertical slides
            if(gamepad1.dpad_up){
                slide = robot.SLIDE_HIGH_RUNG;
            }else if (gamepad1.dpad_down){
                secondLeg = 1;
                slide = robot.SLIDE_START;
            } else if (gamepad1.dpad_left){
                slide = robot.SLIDE_HIGH_BASKET;
            }

            robot.setVerticalPower(secondLeg);
            robot.SetSlidePosition(slide);

            if (gamepad2.x) {
                inClawPitch = 1;
            }
            else if (gamepad2.b) {
                extendOffset = 0.15;
            }

            if (gamepad2.dpad_left) {
                extendOffset = 0;

            } else if (gamepad2.dpad_right) {
                extendOffset = 1;
                inClawPitch = 0;
            }
            // if gamepad2 moves his left stick up/down more than 5% of maximum movement...
            // the higher the number the more minor movements it will detect ( to get to 0.5)
            else if (Math.round(Math.abs(gamepad2.left_stick_y*10)) != 0){
                // set target position to previous distance +/- fudge amount
                extendOffset = -gamepad2.left_stick_y/10;

                if (-gamepad2.left_stick_y == -1){
                    inClawPitch = 0;
                }
            }




            robot.setHorizontalPosition(inClawPitch);
            robot.setIntakePosition(extendOffset);

            extendOffset = 0;

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
            telemetry.addData("HSlidePower", "%.2f", hslide);
            telemetry.addData("Vertical Power", "%.2f", secondLeg);
            telemetry.addData("Horizontal Position", "%.2f", inClawPitch);
            telemetry.addData("Hand Position",  "Offset = %.2f", extendOffset);
            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            pinchTimer++;
            outPinchTimer++;
            sleep(50);
        }
    }
}
