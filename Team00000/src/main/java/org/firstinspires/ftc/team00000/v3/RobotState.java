package org.firstinspires.ftc.team00000.v3;

/**
 * RobotState applies high-level articulation for both TeleOp and Auto.
 * RobotState.applyState applies those states to RobotHardware using setPosition methods
 */
public class RobotState {

    public enum State {
        WALL_INTAKE,     // Intake specimen from wall
        SUB_INTAKE,      // Intake sample from submersible
        HIGH_SCORE,      // Score specimen in high chamber
        STOWED           // Retracted/neutral position
    }

    /**
     * Applies a given RobotState to the current robot hardware configuration
     * by setting servo and motor targets accordingly
     */
    public static void applyState(RobotHardware robot, State state) {
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
                break;
        }
    }
}
