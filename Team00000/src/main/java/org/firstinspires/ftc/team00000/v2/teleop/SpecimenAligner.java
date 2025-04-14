package org.firstinspires.ftc.team00000.v2.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team00000.v2.util.AngleServoController;
import org.firstinspires.ftc.team00000.v2.vision.ColorVisionSubsystem;

@TeleOp(name = "Specimen Aligner Test", group = "Test")
@Config
public class SpecimenAligner extends LinearOpMode {

    // Dashboard tunable parameters
    public static double servoCenter = 0.5;
    public static double angleToPositionGain = 1.0 / 180.0;
    public static double motionThresholdDeg = 2.0;
    public static long updateIntervalMs = 50;
    public static double smoothingAlpha = 0.3;
    public static double minPos = 0.0;
    public static double maxPos = 1.0;

    @Override
    public void runOpMode() {
        // Vision system
        WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
        ColorVisionSubsystem vision = new ColorVisionSubsystem(cam);
        FtcDashboard.getInstance().startCameraStream(vision.getPortal(), 0);

        // Servo
        Servo wrist = hardwareMap.get(Servo.class, "wrist_drive");
        AngleServoController controller = new AngleServoController(wrist, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            vision.update();

            // Update controller parameters from dashboard
            controller.setServoCenter(servoCenter);
            controller.setGain(angleToPositionGain);
            controller.setThreshold(motionThresholdDeg);
            controller.setSmoothingAlpha(smoothingAlpha);
            controller.setUpdateInterval(updateIntervalMs);
            controller.setMinMax(minPos, maxPos);

            if (vision.hasTarget()) {
                double angleError = vision.getAngleErrorToVertical();
                controller.update(angleError);
            }

            telemetry.addData("Has Target?", vision.hasTarget());
            telemetry.addData("Angle", vision.getAngle());
            telemetry.addData("Area", vision.getArea());
            telemetry.update();

            sleep(20);
        }

        vision.stop();
    }
}
