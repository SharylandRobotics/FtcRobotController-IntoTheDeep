package org.firstinspires.ftc.team00000.v2.util;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AngleServoController {

    private final Servo servo;
    private final Telemetry telemetry;

    private double servoCenter = 0.5;
    private double angleToPositionGain = 1.0 / 180.0; // maps ±90 deg to ±0.5 servo range
    private double minPos = 0.0;
    private double maxPos = 1.0;
    private double motionThresholdDeg = 2.0;
    private long updateIntervalMs = 50;

    private double currentPosition = 0.5;
    private long lastUpdateTime = 0;

    private Double smoothedError = null;
    private double smoothingAlpha = 0.3;

    public AngleServoController(Servo servo, Telemetry telemetry) {
        this.servo = servo;
        this.telemetry = telemetry;
        this.currentPosition = servoCenter;
        this.servo.setPosition(currentPosition);
    }

    public void update(double angleError) {
        long now = System.currentTimeMillis();
        if (now - lastUpdateTime < updateIntervalMs) return;
        lastUpdateTime = now;

        // Smoothing (EMA)
        if (smoothedError == null) smoothedError = angleError;
        smoothedError = smoothingAlpha * angleError + (1 - smoothingAlpha) * smoothedError;

        if (Math.abs(smoothedError) < motionThresholdDeg) return;

        double delta = smoothedError * angleToPositionGain;
        double position = clamp(servoCenter - delta, minPos, maxPos);
        servo.setPosition(position);

        if (telemetry != null) {
            telemetry.addData("Servo Pos", position);
            telemetry.addData("Angle Err", angleError);
            telemetry.addData("Smoothed Err", smoothedError);
            telemetry.update();
        }
    }


    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    // Expose tunables (for FTC Dashboard later)
    public void setServoCenter(double center) { this.servoCenter = center; }
    public void setGain(double gain) { this.angleToPositionGain = gain; }
    public void setMinMax(double min, double max) { this.minPos = min; this.maxPos = max; }
    public void setThreshold(double deg) { this.motionThresholdDeg = deg; }
    public void setSmoothingAlpha(double alpha) { this.smoothingAlpha = alpha; }
    public void setUpdateInterval(long ms) { this.updateIntervalMs = ms; }
}