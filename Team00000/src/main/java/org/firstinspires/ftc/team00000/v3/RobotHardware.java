package org.firstinspires.ftc.team00000.v3;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team00000.v3.vision.ColorVisionSubsystem;

public class RobotHardware {

    // Declare OpMode members
    private final LinearOpMode myOpMode; // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects (Make them private so that they CANT be accessed externally)
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive, pivotDrive, slideDrive;
    private Servo clawDrive, wristDrive, armDrive;
    private IMU imu;
    private ColorVisionSubsystem colorVision;

    public double leftFrontPower, leftBackPower, rightFrontPower, rightBackPower;

    public double DRIVE_SPEED, TURN_SPEED, HEADING_THRESHOLD;
    public double COUNTS_PER_MOTOR_REV, DRIVE_GEAR_REDUCTION, WHEEL_DIAMETER_INCHES, COUNTS_PER_INCH;
    public double P_TURN_GAIN, P_DRIVE_GAIN;
    public double PIVOT_TICKS_PER_DEGREE, SLIDE_TICKS_PER_REV, TOLERANCE_TICKS;
    public double PIVOT_MINIMUM, PIVOT_SCORE;
    public double SLIDE_MINIMUM, SLIDE_MAXIMUM;
    public double CLAW_OPEN, CLAW_CLOSE;
    public double WRIST_LEFT, WRIST_CENTER, WRIST_RIGHT;
    public double ARM_ACTIVE, ARM_MID, ARM_INACTIVE;
    public double pivotPosition, slidePosition;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opmode) { myOpMode = opmode; }

    /**
     * Initialize all the robots' hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All the hardware devices are accessed via the hardware map and initialized.
     */
    public void init(){

        // Define and initialize ALL installed motors (note: need to use reference to the actual OpMode).
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class,"left_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");
        pivotDrive = myOpMode.hardwareMap.get(DcMotor.class, "pivot_drive");
        slideDrive = myOpMode.hardwareMap.get(DcMotor.class, "slide_drive");
        clawDrive = myOpMode.hardwareMap.get(Servo.class, "claw_drive");
        wristDrive = myOpMode.hardwareMap.get(Servo.class, "wrist_drive");
        armDrive = myOpMode.hardwareMap.get(Servo.class, "arm_drive");

        /*
        These constants define the desired driving/control characteristics. They can/should be tweaked to suit the specific
        robot drive train.
         */
        DRIVE_SPEED = 0.4; // Maximum autonomous driving speed for better distance accuracy.
        TURN_SPEED = 0.2; // Maximum autonomous turning speed for better rotational accuracy.
        HEADING_THRESHOLD = 1.0; // How close must the heading get to the target before moving to next step.
        // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.

        /*
        Define the Proportional control coefficient (or GAIN) for "heading control". We define one value when Turning
        (large errors), another when Strafing (medium errors), and the other is used when Driving straight (small errors).
        Increase these numbers if the heading does not correct strongly enough (e.g. a heavy robot or using tracks).
        Decrease these numbers if the heading does not settle on the correct value (e.g. very agile robot with omni wheels).
         */
        P_TURN_GAIN = 0.02; // Larger is more responsive, but also less stable.
        P_DRIVE_GAIN = 0.03; // Larger is more responsive, but also less stable.

        /*
        Calculate the COUNTS_PER_INCH for your specific drive train. Go to your motor vendor website to determine your
        motor's COUNT_PER_MOTOR_REV. For external drive gearing set DRIVE_GEAR_REDUCTION as needed. For example, use a
        value of 2.0 for a 12-tooth spur driving a 24-tooth spur gear. This is gearing DOWN for less speed and more
        torque. For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of the wheel
        rotation.
         */
        COUNTS_PER_MOTOR_REV = 537.7; // goBILDA
        DRIVE_GEAR_REDUCTION =  1.0; // No external gearing
        WHEEL_DIAMETER_INCHES = 3.77953; // goBILDA (96mm Diameter)
        COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

        /*
        28: (number of encoder ticks per rotation of the bare motor)
        (((1+(46.0/17.0))) * (1+(46.0/11.0))): (exact gear ratio of the 312RPM Yellow Jacket Motor)
        (((1+(46.0/17.0))) * (1+(46.0/11.0))) * (1+(46.0/11.0)):  (exact gear ratio of the 60RPM Yellow Jacket Motor)
        / 360.0 (ticks per degree, not per rotation)
        */
        PIVOT_TICKS_PER_DEGREE = 28 * 99.5 / 360.0;
        SLIDE_TICKS_PER_REV = 28 * 19.2;
        TOLERANCE_TICKS = 10.0;

        PIVOT_MINIMUM = 8 * PIVOT_TICKS_PER_DEGREE;
        PIVOT_SCORE = 60 * PIVOT_TICKS_PER_DEGREE;

        SLIDE_MINIMUM = 0 * SLIDE_TICKS_PER_REV;
        SLIDE_MAXIMUM = 4 * SLIDE_TICKS_PER_REV;

        CLAW_CLOSE = 0.51;
        CLAW_OPEN = 0.2;

        WRIST_LEFT = 0.75;
        WRIST_CENTER = 0.5;
        WRIST_RIGHT = 0.25;

        ARM_ACTIVE = 0.1666;
        ARM_MID = 0.5;
        ARM_INACTIVE = 0.8333;

        /*
         Define how the hub is mounted on the robot to get the correct Yaw, Pitch, and Roll values. There are two input
         parameters required to fully specify the orientation. (1) the first parameter specifies the direction of the
         printed logo on the hub is pointing. (2) the second parameter specifies the direction the USB connector on the
         hub is pointing. All directions are relative to the robot, and left/right is as-viewed from behind the robot.
         */
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        // Reset the IMU when initializing the hardware class
        imu.resetYaw();

        /*
         Most robots need the motors on one side to be reversed to drive forward. The motor reversals shown here are
         for a "direct drive" robot (the wheels turn in the same direction as the motor shaft). If your robot has
         additional gear reductions or uses a right-angled drive, it is important to ensure that your motors are turning
         in the correct direction. So, start out with the reversals here, BUT when you first test your robot, push the
         left joystick forward and observe the wheels turn. Reverse the direction (flip FORWARD <-> REVERSE) of any
         wheel that runs backward. Keep testing until ALL the wheels move the robot forward when you push the left
         joystick forward.
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        pivotDrive.setDirection(DcMotor.Direction.REVERSE);
        slideDrive.setDirection(DcMotor.Direction.REVERSE);

        // Ensure the robot is stationary. Reset the encoders and set the motors to BREAK mode
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set the encoders for closed loop speed control, and reset the heading.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the TargetPosition to 0. Then we'll set the RunMode to RUN_TO_POSITION, and we'll ask it to stop and reset.
        pivotDrive.setTargetPosition(0);
        slideDrive.setTargetPosition(0);
        pivotDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start
        while (myOpMode.opModeInInit()) {
            myOpMode.telemetry.addData("Status", "Hardware Initialized");
            myOpMode.telemetry.update();
        }
    }

    /**
     * Calculate the motor powers required to achieve the requested robot motions:
     * Drive (Axial motion), Strafe (Lateral motion), and Turn (Yaw motion)
     * Then send these power levels to the motors.
     *
     * @param drive     Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     * @param strafe    Right/Left driving power (-1.0 to 1.0) +ve is right
     * @param turn      Right/Left turning power (-1.0 to 1.0) +ve is clockwise
     */
    public void driveFieldCentric(double drive, double strafe, double turn) {

        double max;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot rotation
        double strafeRotation = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
        double driveRotation = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

        /*
         Combine the joystick requests for each axis-motion to determine each wheel's power. Set up a variable for each
         drive wheel to save the power level for telemetry. Denominator is the largest motor power (absolute value) or
         1. This ensures all the powers maintain the same ratio, but only when at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(driveRotation) + Math.abs(strafeRotation) + Math.abs(turn), 1);
        leftFrontPower = (driveRotation + strafeRotation + turn) / denominator;
        leftBackPower = (driveRotation - strafeRotation + turn) / denominator;
        rightFrontPower = (driveRotation - strafeRotation - turn) / denominator;
        rightBackPower = (driveRotation + strafeRotation - turn) / denominator;

        // Normalize the values so no wheel power exceeds 100%.
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        // Use existing function to drive all wheels.
        setDrivePower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }

    /**
     * Pass the requested wheel motor power to the appropriate hardware drive motors.
     *
     * @param leftFrontWheel    Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     * @param leftBackWheel    Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     * @param rightFrontWheel   Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     * @param rightBackWheel   Forward/Reverse driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double leftFrontWheel, double leftBackWheel, double rightFrontWheel, double rightBackWheel) {
        //Output the values to the motor drives.
        leftFrontDrive.setPower(leftFrontWheel);
        leftBackDrive.setPower(leftBackWheel);
        rightFrontDrive.setPower(rightFrontWheel);
        rightBackDrive.setPower(rightBackWheel);
    }

    // ******************** Articulation control functions ********************

    /**
     * set the target position of our arm to match the variables that was selected by the driver. We also set the target
     * velocity (speed) the motor runs at, and use setMode to run it.
     */
    public void setPivotPosition(double position) {
        pivotDrive.setTargetPosition((int) (position));
        ((DcMotorEx) pivotDrive).setVelocity(2100);
        pivotDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * set the target position of our arm to match the variables that was selected by the driver. We also set the target
     * velocity (speed) the motor runs at, and use setMode to run it.
     */
    public void setSlidePosition(double position) {
        slideDrive.setTargetPosition((int) (position));
        ((DcMotorEx) slideDrive).setVelocity(2100);
        slideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Send the arm-servo to position
     *
     * @param position  (0.0 to 1.0) +ve is retracted
     */
    public void setArmPosition(double position) {
        armDrive.setPosition(position);
    }

    /**
     * Send the wrist-servo to position
     *
     * @param position  (0.0 to 1.0) +ve is clockwise
     */
    public void setWristPosition(double position) {
        wristDrive.setPosition(position);
    }

    /**
     * Send the claw-servo to position
     *
     * @param position  (0.0 to 1.0) +ve is open
     */
    public void setClawPosition(double position) {
        clawDrive.setPosition(position);
    }

}