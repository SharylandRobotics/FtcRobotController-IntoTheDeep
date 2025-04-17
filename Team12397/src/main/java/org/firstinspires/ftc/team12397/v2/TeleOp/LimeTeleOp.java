package org.firstinspires.ftc.team12397.v2.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.team12397.v2.RobotHardware;
import org.firstinspires.ftc.team12397.visionSystems.LLtdc;
import org.firstinspires.ftc.team12397.visionSystems.TdcReturnObject;

import java.util.List;

@TeleOp(name="LimeTeleOp", group="Robot")

public class LimeTeleOp extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private final double COUNTS_PER_MOTOR_REV = 537.7;
    private final double WHEEL_DIAMETER_INCHES = 3.77953;
    private final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI);

    private int leftFrontTarget;
    private int leftBackTarget;
    private int rightFrontTarget;
    private int rightBackTarget;

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight-rfc");
        leftFront = this.hardwareMap.get(DcMotor.class, "left_front");
        leftBack = this.hardwareMap.get(DcMotor.class, "left_back");
        rightFront = this.hardwareMap.get(DcMotor.class, "right_front");
        rightBack = this.hardwareMap.get(DcMotor.class, "right_back");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        LLtdc tdc = new LLtdc(limelight);
        TdcReturnObject returnObj;

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(6);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();
        limelight.pipelineSwitch(6);

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (limelight.getStatus().getPipelineIndex() != 6){
                limelight.pipelineSwitch(6);
            }
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                // Access general information
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);

                if (result.isValid()) {
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("txnc", result.getTxNC());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("tync", result.getTyNC());

                    telemetry.addData("Botpose", botpose.toString());

                    // Access detector results
                    List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                    for (LLResultTypes.DetectorResult dr : detectorResults) {
                        telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                    }
                    tdc.parseDetectorResults(detectorResults);
                    returnObj = tdc.getTdcReturn();
                    telemetry.addData(String.valueOf(returnObj.getRobotYCorrection(DistanceUnit.INCH)), ": y correction");
                    telemetry.addData(String.valueOf(returnObj.getRobotXCorrection(DistanceUnit.INCH)), ": x correction");
                    if (gamepad1.a && tdc.getScanSuccess() && wheelsDone()){
                        driveEncoder(0.4, returnObj.getRobotYCorrection(DistanceUnit.INCH), returnObj.getRobotYCorrection(DistanceUnit.INCH)
                                , returnObj.getRobotYCorrection(DistanceUnit.INCH), returnObj.getRobotYCorrection(DistanceUnit.INCH) );
                    } else if (gamepad1.y && tdc.getScanSuccess() && wheelsDone()){
                        driveEncoder(0.4, returnObj.getRobotXCorrection(DistanceUnit.INCH), -returnObj.getRobotXCorrection(DistanceUnit.INCH)
                                , -returnObj.getRobotXCorrection(DistanceUnit.INCH), returnObj.getRobotXCorrection(DistanceUnit.INCH));
                    }
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }

            telemetry.update();
        }
        limelight.stop();
    }

    public void driveEncoder(double speed, double leftFrontInches, double leftBackInches, double rightFrontInches, double rightBackInches) {
        // drives only while myOpMode is active
        if (this.opModeIsActive()) {

            //determine new target position
            leftFrontTarget = leftFront.getCurrentPosition() + (int) (leftFrontInches * COUNTS_PER_INCH);
            leftBackTarget = leftBack.getCurrentPosition() + (int) (leftBackInches * COUNTS_PER_INCH);
            rightFrontTarget = rightFront.getCurrentPosition() + (int) (rightFrontInches * COUNTS_PER_INCH);
            rightBackTarget = rightBack.getCurrentPosition() + (int) (rightBackInches * COUNTS_PER_INCH);

            leftFront.setTargetPosition(leftFrontTarget);
            leftBack.setTargetPosition(leftBackTarget);
            rightFront.setTargetPosition(rightFrontTarget);
            rightBack.setTargetPosition(rightBackTarget);

            //turn on RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the time and start motion
            setDrivePower(Math.abs(speed), Math.abs(speed), Math.abs(speed), Math.abs(speed));

            while ((opModeIsActive() &&
                    (leftFront.isBusy() && leftBack.isBusy() &&
                            rightFront.isBusy() && rightBack.isBusy()))) {

                //display it for driver

                telemetry.addData("Running to ", " %7d :%7d :%7d :%7d",
                        leftFront, leftBack, rightFront, rightBack);
                telemetry.addData("Currently at ", "%7d ;%7d :%7d :%7d",
                        leftFront.getCurrentPosition(), leftBack.getCurrentPosition(),
                        rightFront.getCurrentPosition(), rightBack.getCurrentPosition());
                telemetry.update();
            }

            setDrivePower(0, 0, 0, 0);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }
    public void setDrivePower(double leftFrontWheel, double leftBackWheel, double rightFrontWheel, double rightBackWheel) {
        //Output the values to the motor drives.
        leftFront.setPower(leftFrontWheel);
        leftBack.setPower(leftBackWheel);
        rightFront.setPower(rightFrontWheel);
        rightBack.setPower(rightBackWheel);
    }

    private boolean wheelsDone(){
        return !(leftFront.isBusy() || leftBack.isBusy() || rightFront.isBusy() || rightBack.isBusy());
    }
}
