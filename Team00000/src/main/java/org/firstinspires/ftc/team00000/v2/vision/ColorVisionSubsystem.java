package org.firstinspires.ftc.team00000.v2.vision;

import android.util.Size;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.List;

@Config
public class ColorVisionSubsystem {

    private final VisionPortal visionPortal;
    private final ColorBlobLocatorProcessor processor;
    private RotatedRect currentTarget;

    // Dashboard tunable
    public static int minArea = 50;
    public static int maxArea = 20000;
    public static double minAspectRatio = 1.25;
    public static ColorRange targetColorRange = ColorRange.BLUE;

    private static final double SMOOTHING_ALPHA = 0.2;
    private Double smoothedAngle = null;

    public ColorVisionSubsystem(WebcamName webcam) {
        // Setup processor
        processor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(targetColorRange)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(true)
                .setBlurSize(5)
                .setRoi(ImageRegion.entireFrame())
                .build();

        // Setup portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(320, 240))
                .addProcessor(processor)
                .build();
    }

    public void update() {
        List<ColorBlobLocatorProcessor.Blob> blobs = processor.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(minArea, maxArea, blobs);

        if (!blobs.isEmpty()) {
            for (ColorBlobLocatorProcessor.Blob blob : blobs) {
                RotatedRect rawRect = blob.getBoxFit();
                double width = rawRect.size.width;
                double height = rawRect.size.height;
                double aspectRatio = Math.max(width, height) / Math.min(width, height);

                if (aspectRatio < minAspectRatio) continue;

                double angle = rawRect.angle;
                if (width < height) angle += 90;
                if (angle < 0) angle += 180;

                if (smoothedAngle == null) smoothedAngle = angle;
                smoothedAngle = SMOOTHING_ALPHA * angle + (1 - SMOOTHING_ALPHA) * smoothedAngle;

                currentTarget = new RotatedRect(rawRect.center, rawRect.size, smoothedAngle);
                return;
            }
        }

        currentTarget = null;
    }

    public boolean hasTarget() {
        return currentTarget != null;
    }

    public double getAngle() {
        return hasTarget() ? currentTarget.angle : 0;
    }

    public Point getCenter() {
        return hasTarget() ? currentTarget.center : new Point(0, 0);
    }

    public double getArea() {
        return hasTarget() ? currentTarget.size.area() : 0.0;
    }

    public RotatedRect getTargetRect() {
        return currentTarget;
    }

    public double getAngleErrorToVertical() {
        return hasTarget() ? currentTarget.angle - 90 : 0;
    }

    public VisionPortal getPortal() {
        return visionPortal;
    }

    public void stop() {visionPortal.close();
    }
}
