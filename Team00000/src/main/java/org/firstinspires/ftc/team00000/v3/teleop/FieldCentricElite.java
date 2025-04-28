package org.firstinspires.ftc.team00000.v3.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team00000.v3.RobotHardware;

@TeleOp(name = "Field Centric Elite", group = "TeleOp")
public class FieldCentricElite extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    // Toggle states
    private boolean clawToggleState = false;
    private boolean lastClawButton = false;

    @Override
    public void runOpMode() {
        double drive;
        double strafe;
        double turn;

        robot.init();

        robot.setClawPosition(robot.CLAW_OPEN);
        robot.setWristPosition(robot.WRIST_CENTER);
        robot.setArmPosition(robot.ARM_INACTIVE);
        robot.setPivotPosition(robot.PIVOT_MINIMUM);
        robot.setSlidePosition(robot.SLIDE_MINIMUM);

        waitForStart();

        while (opModeIsActive()) {

            // -------------------- Drive Control --------------------
            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x * 1.1;
            turn = gamepad1.right_stick_x;

            robot.driveFieldCentric(drive, strafe, turn);

            // -------------------- Claw Toggle (Right Bumper) --------------------
            boolean currentClawButton = gamepad1.right_bumper;
            if (currentClawButton && !lastClawButton) {
                clawToggleState = !clawToggleState;
                if (clawToggleState) {
                    robot.setClawPosition(robot.CLAW_CLOSE);
                } else {
                    robot.setClawPosition(robot.CLAW_OPEN);
                }
            }
            lastClawButton = currentClawButton;

            // -------------------- Wrist Manual Control (Triggers) --------------------
            if (gamepad1.right_trigger > 0.5) {
                robot.setWristPosition(robot.WRIST_RIGHT);
            } else if (gamepad1.left_trigger > 0.5) {
                robot.setWristPosition(robot.WRIST_LEFT);
            } else {
                robot.setWristPosition(robot.WRIST_CENTER);
            }

            // -------------------- Manual Arm Control (Dpad Up/Down) --------------------
            if (gamepad1.dpad_up) {
                robot.setArmPosition(robot.ARM_INACTIVE);
            } else if (gamepad1.dpad_down) {
                robot.setArmPosition(robot.ARM_ACTIVE);
            }

            // -------------------- Manual Slide Control (Dpad Left/Right) --------------------
            if (gamepad1.dpad_right) {
                robot.setSlidePosition(robot.SLIDE_MAXIMUM);
            } else if ( gamepad1.dpad_left) {
                robot.setSlidePosition(robot.SLIDE_MINIMUM);
            }

            // -------------------- Pivot Control (A Button) --------------------
            if (gamepad1.a) {
                robot.setPivotPosition(robot.PIVOT_SCORE);
            } else if (gamepad1.b);

            // -------------------- Macro: Score Position (B Button) --------------------
            if (gamepad1.b) {
                robot.setArmPosition(robot.ARM_ACTIVE);
                robot.setPivotPosition(robot.PIVOT_SCORE);
                robot.setSlidePosition(robot.SLIDE_MAXIMUM);
            }

            // -------------------- Macro: Stow/Reset (X Button) --------------------
            if (gamepad1.x) {
                robot.setClawPosition(robot.CLAW_CLOSE); // Optional: close claw during retract
                robot.setSlidePosition(robot.SLIDE_MINIMUM);
                robot.setPivotPosition(robot.PIVOT_MINIMUM);
                robot.setArmPosition(robot.ARM_INACTIVE);
            }

            telemetry.addData("Drive Power", "%.2f", drive);
            telemetry.addData("Strafe Power", "%.2f", strafe);
            telemetry.addData("Turn Power", "%.2f", turn);
            telemetry.update();
        }
    }
}