package org.firstinspires.ftc.team00000.v3;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * RobotHardware handles all robot hardware initialization and control.
 * It provides direct drive access and Road Runner compatible action interfaces.
 */
public class RobotHardware {

    private final LinearOpMode myOpMode;

    // Drive and mechanism hardware
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive, pivotDrive, slideDrive;
    private Servo clawDrive, wristDrive, armDrive;
    private IMU imu;

    // Drive power values
    public double leftFrontPower, leftBackPower, rightFrontPower, rightBackPower;

    // Constants and tunables
    public double DRIVE_SPEED, TURN_SPEED, HEADING_THRESHOLD;
    public double COUNTS_PER_MOTOR_REV, DRIVE_GEAR_REDUCTION, WHEEL_DIAMETER_INCHES, COUNTS_PER_INCH;
    public double P_TURN_GAIN, P_DRIVE_GAIN;
    public double PIVOT_TICKS_PER_DEGREE, SLIDE_TICKS_PER_REV, TOLERANCE_TICKS;

    // Preset positions
    public double PIVOT_MINIMUM, PIVOT_SPECIMEN, PIVOT_SCORE, PIVOT_MAXIMUM;
    public double SLIDE_MINIMUM, SLIDE_SCORE, SLIDE_MAXIMUM;
    public double CLAW_OPEN, CLAW_CLOSE;
    public double WRIST_LEFT, WRIST_CENTER, WRIST_RIGHT;
    public double ARM_ACTIVE, ARM_SPECIMEN, ARM_SCORE, ARM_INACTIVE;

    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initializes all robot hardware. Should be called during init() phase.
     */
    public void init() {
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");
        pivotDrive = myOpMode.hardwareMap.get(DcMotor.class, "pivot_drive");
        slideDrive = myOpMode.hardwareMap.get(DcMotor.class, "slide_drive");

        clawDrive = myOpMode.hardwareMap.get(Servo.class, "claw_drive");
        wristDrive = myOpMode.hardwareMap.get(Servo.class, "wrist_drive");
        armDrive = myOpMode.hardwareMap.get(Servo.class, "arm_drive");

        // Drive and heading config
        DRIVE_SPEED = 0.4;
        TURN_SPEED = 0.2;
        HEADING_THRESHOLD = 1.0;

        P_TURN_GAIN = 0.02;
        P_DRIVE_GAIN = 0.03;

        COUNTS_PER_MOTOR_REV = 537.7;
        DRIVE_GEAR_REDUCTION = 1.0;
        WHEEL_DIAMETER_INCHES = 3.77953;
        COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

        PIVOT_TICKS_PER_DEGREE = 28 * 99.5 / 360.0;
        SLIDE_TICKS_PER_REV = 28 * 19.2;
        TOLERANCE_TICKS = 10.0;

        PIVOT_MINIMUM = 9 * PIVOT_TICKS_PER_DEGREE;
        PIVOT_SPECIMEN = 27 * PIVOT_TICKS_PER_DEGREE;
        PIVOT_SCORE = 42 * PIVOT_TICKS_PER_DEGREE;
        PIVOT_MAXIMUM = 90 * PIVOT_TICKS_PER_DEGREE;

        SLIDE_MINIMUM = 0 * SLIDE_TICKS_PER_REV;
        SLIDE_SCORE = 3 * SLIDE_TICKS_PER_REV;
        SLIDE_MAXIMUM = 4 * SLIDE_TICKS_PER_REV;

        CLAW_CLOSE = 0.55;
        CLAW_OPEN = 0.2;

        WRIST_LEFT = 0.75;
        WRIST_CENTER = 0.5;
        WRIST_RIGHT = 0.25;

        ARM_ACTIVE = 0.1666;
        ARM_SPECIMEN = 0.425;
        ARM_SCORE = 0.6666;
        ARM_INACTIVE = 0.8333;

        // IMU setup
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(imuParams);
        imu.resetYaw();

        // Motor directions and modes
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        pivotDrive.setDirection(DcMotor.Direction.REVERSE);
        slideDrive.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor motor : new DcMotor[]{leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive}) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        for (DcMotor motor : new DcMotor[]{pivotDrive, slideDrive}) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        while (myOpMode.opModeInInit()) {
            myOpMode.telemetry.addData("Status", "Hardware Initialized");
            myOpMode.telemetry.update();
        }
    }

    /**
     * Converts joystick drive + strafe + turn into field-relative drive signals using IMU heading.
     */
    public void driveFieldCentric(double drive, double strafe, double turn) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double strafeRotation = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
        double driveRotation = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);
        double denominator = Math.max(Math.abs(driveRotation) + Math.abs(strafeRotation) + Math.abs(turn), 1);

        leftFrontPower = (driveRotation + strafeRotation + turn) / denominator;
        leftBackPower = (driveRotation - strafeRotation + turn) / denominator;
        rightFrontPower = (driveRotation - strafeRotation - turn) / denominator;
        rightBackPower = (driveRotation + strafeRotation - turn) / denominator;

        double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower)),
                Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower)));

        if (max > 1.0) {
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        setDrivePower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }

    public void setDrivePower(double lf, double lb, double rf, double rb) {
        leftFrontDrive.setPower(lf);
        leftBackDrive.setPower(lb);
        rightFrontDrive.setPower(rf);
        rightBackDrive.setPower(rb);
    }

    public void setPivotPosition(double position) {
        position = Math.max(PIVOT_MINIMUM, Math.min(PIVOT_MAXIMUM, position));
        pivotDrive.setTargetPosition((int) position);
        pivotDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) pivotDrive).setVelocity(500);
    }

    public void setSlidePosition(double position) {
        position = Math.max(SLIDE_MINIMUM, Math.min(SLIDE_MAXIMUM, position));
        slideDrive.setTargetPosition((int) position);
        slideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) slideDrive).setVelocity(2100);
    }

    public void setArmPosition(double position) {
        armDrive.setPosition(position);
    }

    public void setWristPosition(double position) {
        wristDrive.setPosition(position);
    }

    public void setClawPosition(double position) {
        clawDrive.setPosition(position);
    }

    // RR2 Action wrappers for asynchronous logic

    public Action movePivot(double position) {
        return packet -> {
            setPivotPosition(position);
            return Math.abs(pivotDrive.getCurrentPosition() - position) > TOLERANCE_TICKS;
        };
    }

    public Action moveSlide(double position) {
        return packet -> {
            setSlidePosition(position);
            return Math.abs(slideDrive.getCurrentPosition() - position) > TOLERANCE_TICKS;
        };
    }

    public Action moveArm(double position) {
        return packet -> {
            setArmPosition(position);
            return false;
        };
    }

    public Action moveWrist(double position) {
        return packet -> {
            setWristPosition(position);
            return false;
        };
    }

    public Action moveClaw(double position) {
        return packet -> {
            setClawPosition(position);
            return false;
        };
    }

    public void stopAllMotion() {
        setDrivePower(0, 0, 0, 0);
        ((DcMotorEx) pivotDrive).setVelocity(0);
        ((DcMotorEx) slideDrive).setVelocity(0);
    }
}
