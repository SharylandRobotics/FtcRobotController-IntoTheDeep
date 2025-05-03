package org.firstinspires.ftc.team00000.v3.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team00000.v3.RobotHardware;
import org.firstinspires.ftc.team00000.v3.RobotState;

/**
 * Field-centric TeleOp that integrates state-based articulation logic
 * with field-relative drive using IMU yaw angle.
 */
@TeleOp(name = "Field Centric", group = "TeleOp")
public class FieldCentric extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    private RobotState.State currentState = RobotState.State.STOWED;

    private boolean clawToggleState = false;
    private boolean lastClawButton = false;

    @Override
    public void runOpMode() {
        double drive;
        double strafe;
        double turn;

        robot.init();

        RobotState.applyState(robot, RobotState.State.STOWED);

        waitForStart();

        while (opModeIsActive()) {

            // -------------------- Drive Input --------------------
            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x * 1.1;
            turn = gamepad1.right_stick_x;

            robot.driveFieldCentric(drive, strafe, turn);

            // -------------------- Claw Toggle (Right Bumper) --------------------
            boolean currentClawButton = gamepad1.right_bumper;
            if (currentClawButton && !lastClawButton) {
                clawToggleState = !clawToggleState;
                robot.setClawPosition(clawToggleState ? robot.CLAW_CLOSE : robot.CLAW_OPEN);
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

            // -------------------- Macro State Controls (ABXY) --------------------
            if (gamepad1.b && currentState != RobotState.State.WALL_INTAKE) {
                currentState = RobotState.State.WALL_INTAKE;
                RobotState.applyState(robot, currentState);
            } else if (gamepad1.a && currentState != RobotState.State.SUB_INTAKE) {
                currentState = RobotState.State.SUB_INTAKE;
                RobotState.applyState(robot, currentState);
            } else if (gamepad1.y && currentState != RobotState.State.HIGH_SCORE) {
                currentState = RobotState.State.HIGH_SCORE;
                RobotState.applyState(robot, currentState);
            } else if (gamepad1.x && currentState != RobotState.State.STOWED) {
                currentState = RobotState.State.STOWED;
                RobotState.applyState(robot, currentState);
            }

            // -------------------- Telemetry Output --------------------
            telemetry.addData("Drive Power", "%.2f", drive);
            telemetry.addData("Strafe Power", "%.2f", strafe);
            telemetry.addData("Turn Power", "%.2f", turn);
            telemetry.addData("Macro State", currentState);
            telemetry.update();
        }
    }
}
