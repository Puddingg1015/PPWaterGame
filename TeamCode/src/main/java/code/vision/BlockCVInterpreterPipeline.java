package code.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BlockCVInterpreterPipeline extends OpenCvPipeline {

    // Detection results storage
    private double orientation;
    private double quadrant;
    private double centerX;
    private double centerY;

    // Tuned HSV thresholds (adjust these for your lighting)
    public static double strictLowH = 110;
    public static double strictHighH = 150;
    public static double strictLowS = 100;
    public static double strictHighS = 255;

    // Visualization
    private Mat output = new Mat();
    private boolean showDebug = true;

    public BlockCVInterpreterPipeline() {
        // Initialize variables
        orientation = -1;
        centerX = -1;
        centerY = -1;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Convert to HSV color space
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Blue color thresholding
        Mat threshold = new Mat();
        Core.inRange(hsv,
                new Scalar(80, 90, 50),    // Lower HSV (Hue, Saturation, Value)
                new Scalar(150, 255, 255),  // Upper HSV
                threshold);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(threshold, contours, hierarchy,
                Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find largest contour
        MatOfPoint largestContour = null;
        double maxArea = 0;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

        if (largestContour != null && maxArea > 100) {
            // Get minimum area rectangle
            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(largestContour.toArray()));

            // Store properties
            this.centerX = rect.center.x;
            this.centerY = rect.center.y;

            // Angle correction logic
            double rawAngle = rect.angle;
            this.orientation = rawAngle < -45 ?
                    (90 + (-rawAngle)) :  // Left tilts
                    (-rawAngle);          // Right tilts
//            this.orientation = rawAngle;

            // Visual debugging
            if (showDebug) {
                Point[] vertices = new Point[4];
                rect.points(vertices);
                List<MatOfPoint> boxContour = new ArrayList<>();
                boxContour.add(new MatOfPoint(vertices));
                Imgproc.drawContours(input, boxContour, -1, new Scalar(0, 255, 0), 2);
                Imgproc.putText(input, String.format("%.1f°", orientation),
                        new Point(rect.center.x + 10, rect.center.y),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 0, 0), 2);
            }
        }

        // Cleanup
        hsv.release();
        threshold.release();
        hierarchy.release();

        // Return processed frame
        input.copyTo(output);
        return output;
    }

    public double getOrientation() {
        return orientation;
    }

    public double getCenterX() {
        return centerX;
    }

    public double getCenterY() {
        return centerY;
    }
}