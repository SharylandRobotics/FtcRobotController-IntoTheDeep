package org.firstinspires.ftc.team12397.v2;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;

    public ElapsedTime runtime = new ElapsedTime();

    public IMU imu = null;

    public double drivePower = 0;
    public double strafePower = 0;
    public double turnPower = 0;

    public double leftFrontPower = 0;
    public double leftBackPower = 0;
    public double rightFrontPower = 0;
    public double rightBackPower = 0;

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotorEx slideMotorL = null;
    private DcMotorEx slideMotorR = null;

    private Servo inClawPitch = null;
    private Servo lExtend = null;
    private Servo rExtend = null;
    public Servo leftOutTake = null;
    public Servo rightOutTake = null;
    private Servo inClawPinch = null;
    public Servo outClawPinch = null;
    private Servo inClawYaw = null;
    public Servo outClawYaw = null;

    public final double OUTTAKE_ALT = 0.12;
    public final double OUTTAKE_BALT = 0.29;
    public final double OUTTAKE_PARALLEL = 0.45;
    public final double SLIDE_ALT = 9;
    public final double OUTTAKE_MAX = 0.8;
    public final double OUTTAKE_MIN = 0.32;
    public final double OUTTAKE_MID = 0.8;

    public final double EXTEND_MAX = 0.955;
    public final double EXTEND_MID = 0.4;
    public final double EXTEND_MIN = 0;

    public final double PITCH_MAX = 1;
    public final double PITCH_MID = 0.65;
    public final double PITCH_MIN = 0.15;

    public final double IN_YAW_MAX = 0.4225;
    public final double IN_YAW_MIN = 0;

    public final double OUT_YAW_MAX = 0.7;
    public final double OUT_YAW_MIN = 0.04;

    public final double SLIDE_RUNG = 16;
    public final double PITCH_TRANS = 0.1;
    public final double EXTEND_TRANS = 0.4;

    public final double OUTTAKE_MAX = 0.8;
    public final double OUTTAKE_MIN = 0.32;
    public final double OUTTAKE_MID = 0.8;

    // ticks
    public double COUNTS_PER_MOTOR_REV = 537.7;
    public double WHEEL_DIAMETER_INCHES = 3.77953;
    public double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI);



    // slide ticks
    public final double SLIDE_TICKS_PER_DEGREE = ((((1+(46./17))) * (1+(46./11))) * 28) / 360;
    public final double TICKS_PER_INCH = (SLIDE_TICKS_PER_DEGREE*360)/(Math.PI*1.5);
    public final double SLIDE_HIGH_BASKET = 2100 * SLIDE_TICKS_PER_DEGREE;
    public final double SLIDE_HANG_RUNG = 5 * TICKS_PER_INCH;

    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

    public final double PITCH_MAX = 1;
    public final double PITCH_MID = 0.65;
    public final double PITCH_MIN = 0.15;

    IMU.Parameters TeleOpParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

    public RobotHardware(LinearOpMode OpMode) {myOpMode = OpMode;}

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All the hardware devices are accessed via the hardware map, and initialized.
     */

    public void init(boolean teleOp) {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_front");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back");
        slideMotorL = myOpMode.hardwareMap.get(DcMotorEx.class, "slideMotorL");
        slideMotorR = myOpMode.hardwareMap.get(DcMotorEx.class, "slideMotorR");



        // To drive forward, most robots need the motor on one side to be reversed, because the axles point on opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels. Gear Reduction or 90 Deg drives mat require direction flips.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        slideMotorL.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotorR.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideMotorL.setTargetPosition(0);
        slideMotorR.setTargetPosition(0);
        slideMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Define and initialize ALL installed servos.
        outClawYaw = myOpMode.hardwareMap.get(Servo.class, "claw_out_yaw");
        inClawPitch = myOpMode.hardwareMap.get(Servo.class, "claw_pitch");
        lExtend = myOpMode.hardwareMap.get(Servo.class, "lextend");
        rExtend = myOpMode.hardwareMap.get(Servo.class, "rextend");
        leftOutTake = myOpMode.hardwareMap.get(Servo.class, "leftOutTake");
        rightOutTake = myOpMode.hardwareMap.get(Servo.class, "rightOutTake");

        inClawPinch = myOpMode.hardwareMap.get(Servo.class, "claw_pinch");
        outClawPinch = myOpMode.hardwareMap.get(Servo.class, "claw_out_pinch");
        inClawYaw = myOpMode.hardwareMap.get(Servo.class, "claw_yaw");

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        imu.initialize(parameters);
        imu.resetYaw();

        if (teleOp) {
            imu.initialize(TeleOpParameters);
            imu.resetYaw();
        }



        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.addData("Heading: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        myOpMode.telemetry.update();
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param Drive     Fwd/Rev driving power(-1.0 to 1.0) +ve is forward
     * @param Turn      Right/Left turning power(-1.0 to 1.0) +ve is CW
     * @param Strafe
     */
    public void driveRobotCentric(double Drive,double Strafe, double Turn){
        //Combine drive and turn for blended motion.
        double leftFrontPower = Drive + Strafe + Turn;
        double leftBackPower = Drive - Strafe + Turn;
        double rightFrontPower = Drive - Strafe - Turn;
        double rightBackPower = Drive + Strafe -  Turn;

        //Scale the values so neither exceed +/-1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0)
        {
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        //Use existing function to drive both wheels.
        setDrivePower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }

    public void driveFieldCentric(double drive, double strafe, double turn){
        drivePower = drive;
        strafePower = strafe;
        turnPower = turn;

        double max;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double strafeRotation = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
        double driveRotation = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);



        leftFrontPower = driveRotation + strafeRotation + turn;
        leftBackPower = driveRotation - strafeRotation + turn;
        rightFrontPower = driveRotation - strafeRotation - turn;
        rightBackPower = driveRotation + strafeRotation - turn;

        max = Math.max(Math.abs(leftBackPower), Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if(max > 1.0){
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        setDrivePower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);


    }

    public void driveEncoder(double speed, double leftFrontInches, double leftBackInches, double rightFrontInches, double rightBackInches){
        // drives only while myOpMode is active
        if(myOpMode.opModeIsActive()){


            //determine new target position
            leftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            leftBackTarget = leftBackDrive.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            rightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            rightBackTarget = rightBackDrive.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

            leftFrontDrive.setTargetPosition(leftFrontTarget - 1);
            leftBackDrive.setTargetPosition(leftBackTarget - 1 );
            rightFrontDrive.setTargetPosition(rightFrontTarget - 1);
            rightBackDrive.setTargetPosition(rightBackTarget - 1 );

            //turn on RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the time and start motion

            runtime.reset();
            setDrivePower(Math.abs(speed), Math.abs(speed), Math.abs(speed), Math.abs(speed));

            while ((myOpMode.opModeIsActive() &&
                    (leftFrontDrive.isBusy() && leftBackDrive.isBusy() &&
                            rightFrontDrive.isBusy() && rightBackDrive.isBusy()))){

                //display it for driver

                myOpMode.telemetry.addData("Running to ", " %7d :%7d :%7d :%7d",
                        leftFrontTarget, leftBackTarget, rightFrontTarget, rightBackTarget);
                myOpMode.telemetry.addData("Currently at ", "%7d ;%7d :%7d :%7d",
                        leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                myOpMode.telemetry.update();
            }

            setDrivePower(0, 0, 0, 0 );
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            myOpMode.sleep(200); //optional pause after each move
        }
    }




    public void setSlidePosition(double slidePosition){
        slideMotorL.setTargetPosition((int)(slidePosition));
        slideMotorR.setTargetPosition((int)(slidePosition));

        slideMotorL.setVelocity(2500);
        slideMotorR.setVelocity(2500);

        slideMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftFrontWheel     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param leftBackWheel     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightFrontWheel    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightBackWheel    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double leftFrontWheel, double leftBackWheel, double rightFrontWheel, double rightBackWheel) {
        //Output the values to the motor drives.
        leftFrontDrive.setPower(leftFrontWheel);
        leftBackDrive.setPower(leftBackWheel);
        rightFrontDrive.setPower(rightFrontWheel);
        rightBackDrive.setPower(rightBackWheel);
    }



    /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed position.
     * the extends are what extend the slides
     * @param position value from 0-1
     */
    public void setExtensionPos(double position){
        //whatever value you subtract from lextend should be added to rextend and vise versa
        position = Math.min(EXTEND_MAX, position);
        position = Math.max(0, position);

        lExtend.setPosition(1 - (position*0.25));
        rExtend.setPosition(0 + (position*0.25));
        // 0.4 extend, 0.3 pitch, 0.8 leg for passing
    }

    public void setInClawPitchPos(double interpretedPos) {
        if (interpretedPos == 1){
            inClawPitch.setPosition(1);

        } else if (interpretedPos == 0){
            inClawPitch.setPosition(0.3);

        } else if (interpretedPos == 0.5) {
            inClawPitch.setPosition(0.5);
        } else {
            // tucked in (-1)
            inClawPitch.setPosition(0.05);
        }
        // 1 is down, 0.3 is passing
    }

    /**
     *
     * @param position 1 is all the way back, 0 is the other way.
     */
    public void setOutTakePos(double position) {
        if (position == 0){
            leftOutTake.setPosition(OUTTAKE_MID);
            rightOutTake.setPosition(1.01 - OUTTAKE_MID);
        } else if (position == 1){
            leftOutTake.setPosition(OUTTAKE_MIN);
            rightOutTake.setPosition(1.0 - OUTTAKE_MIN);
        } else if (position == -1){
            leftOutTake.setPosition(0.55);
            rightOutTake.setPosition(1.01 - 0.55);
        } else {
            leftOutTake.setPosition(0.6);
            rightOutTake.setPosition(1.01 - 0.6);
        }
        // right servo is 0.01 ahead,

        // 0.77 is mid, 1 is back, 0 is in robot
        // 0.45 is perp.
    }

    public void setOutTakeCustom(double pos){
        leftOutTake.setPosition(pos);
        rightOutTake.setPosition(1.01 - pos);
    }

    public void setInClawPitchCustom(double pos){
        pos = Math.max(PITCH_MIN, pos);
        inClawPitch.setPosition(pos);
    }


    public void setInClawYaw(double pos){
        pos = Math.min(IN_YAW_MAX, pos); // we don't filter for (-) bc it will always go to 0.
        inClawYaw.setPosition(pos);
        // 0 is mid, 0.3 is to the left 90 deg!MAX!
    }

    public void setOutClawYaw(double interpretedPos){
        if (interpretedPos == 1) {
            outClawYaw.setPosition(OUT_YAW_MAX);
        } else {
            outClawYaw.setPosition(OUT_YAW_MIN);
        }

        // 0 default, 0.65 is flipped
    }

    /**
     *
     * @param pos 1 is closed, 0 is open
     */
    public void setOutClawPinch(double pos){
        if (pos == 1) {
            outClawPinch.setPosition(0.38);

        } else {
            outClawPinch.setPosition(0);
        }
        // 0.85 closed , 0.46 open

    }
    /**
     *
     * @param pos 1 is closed, 0 is open
     */
    public void setInClawPinch(double pos){
        if (pos == 1) {
            inClawPinch.setPosition(0.35);
        } else {
            inClawPinch.setPosition(0);
        }
        // 0.35 closed , 0 open
    }
}


