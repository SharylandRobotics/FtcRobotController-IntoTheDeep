package org.firstinspires.ftc.team00000.v3.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team00000.v3.RobotHardware;

@TeleOp(name = "Field Centric Elite", group = "TeleOp")
public class FieldCentricElite extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    private enum RobotState {
        WALL_INTAKE, SUB_INTAKE, HIGH_SCORE, STOWED
    }

    // Toggle states
    private boolean clawToggleState = false;
    private boolean lastClawButton = false;
    private RobotState currentState = RobotState.STOWED;

    @Override
    public void runOpMode() {
        double drive;
        double strafe;
        double turn;

        robot.init();

        robot.setClawPosition(robot.CLAW_CLOSE);
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

            // -------------------- Macro State Controls --------------------
            if (gamepad1.b && currentState != RobotState.WALL_INTAKE) {
                setRobotState(RobotState.WALL_INTAKE);
                setRobotState(currentState);
            } else if (gamepad1.a && currentState != RobotState.SUB_INTAKE) {
                setRobotState(RobotState.SUB_INTAKE);
                setRobotState(currentState);
            } else if (gamepad1.y && currentState != RobotState.HIGH_SCORE) {
                setRobotState(RobotState.HIGH_SCORE);
                setRobotState(currentState);
            } else if (gamepad1.x && currentState != RobotState.STOWED) {
                setRobotState(RobotState.STOWED);
                setRobotState(currentState);
            }

            telemetry.addData("Drive Power", "%.2f", drive);
            telemetry.addData("Strafe Power", "%.2f", strafe);
            telemetry.addData("Turn Power", "%.2f", turn);
            telemetry.addData("\nMacro State", currentState);
            telemetry.update();
        }
    }

    private void setRobotState(RobotState state) {
        switch (state) {
            case WALL_INTAKE:
                robot.setClawPosition(robot.CLAW_OPEN);
                robot.setArmPosition(robot.ARM_SPECIMEN);
                robot.setPivotPosition(robot.PIVOT_SPECIMEN);
                robot.setSlidePosition(robot.SLIDE_MINIMUM);
                robot.setWristPosition(robot.WRIST_CENTER);
                break;

            case SUB_INTAKE:
                robot.setClawPosition(robot.CLAW_OPEN);
                robot.setArmPosition(robot.ARM_ACTIVE);
                robot.setPivotPosition(robot.PIVOT_MINIMUM);
                robot.setSlidePosition(robot.SLIDE_SCORE);
                robot.setWristPosition(robot.WRIST_CENTER);
                break;

            case HIGH_SCORE:
                robot.setClawPosition(robot.CLAW_CLOSE);
                robot.setArmPosition(robot.ARM_SCORE);
                robot.setPivotPosition(robot.PIVOT_SCORE);
                robot.setSlidePosition(robot.SLIDE_SCORE);
                robot.setWristPosition(robot.WRIST_CENTER);
                break;

            case STOWED:
                robot.setClawPosition(robot.CLAW_CLOSE);
                robot.setArmPosition(robot.ARM_SCORE);
                robot.setPivotPosition(robot.PIVOT_MINIMUM);
                robot.setSlidePosition(robot.SLIDE_MINIMUM);
                robot.setWristPosition(robot.WRIST_CENTER);
        }
    }
}