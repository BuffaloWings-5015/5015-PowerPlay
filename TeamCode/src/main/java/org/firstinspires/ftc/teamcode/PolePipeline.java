package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PolePipeline extends OpenCvPipeline {
    /*
    YELLOW  = Parking Left
    CYAN    = Parking Middle
    MAGENTA = Parking Right
     */
    Telemetry telemetry;
    public Point position;
    // TOPLEFT anchor point for the bounding box
    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(445, 78);

    // Width and height for the bounding box
    public static int REGION_WIDTH = 300;
    public static int REGION_HEIGHT = 500;


    // Lower and upper boundaries for colors
    private static final Scalar
            lower_yellow_bounds  = new Scalar(20, 120, 120),
            upper_yellow_bounds  = new Scalar(30, 255, 255);


    // Color definitions
    private final Scalar
            YELLOW  = new Scalar(150,255,255),
            CYAN    = new Scalar(0, 255, 255);


    // Percent and mat definitions
    private double yelPercent, cyaPercent, magPercent;
    private Mat yelMat = new Mat(),
            mat = new Mat(),
            yelYelMat = new Mat(),
            blurredMat = new Mat(),
            kernel = new Mat();

    // Anchor point definitions
    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Running variable storing the parking position

    public PolePipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    @Override
    public Mat processFrame(Mat input) {
        // Noise reduction
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Imgproc.blur(mat, blurredMat, new Size(5, 5));

        // Apply Morphology
        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel);

        // Gets channels from given source mat
        Core.inRange(blurredMat, lower_yellow_bounds, upper_yellow_bounds, yelMat);
        List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchey = new Mat();
            Imgproc.findContours(yelMat, contours, hierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            //Drawing the Contours
            MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
            Rect[] boundRect = new Rect[contours.size()];
            Point[] centers = new Point[contours.size()];
            float[][] radius = new float[contours.size()][1];
            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
                centers[i] = new Point();
                Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i]);

            //Adding text to the image
            /*
            Imgproc.putText(input, text, position, font, scale, new Scalar(90,245,255), thickness);

             */
        }
        List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
        for (MatOfPoint2f poly : contoursPoly) {
            contoursPolyList.add(new MatOfPoint(poly.toArray()));
        }

        double maxVal = 0;
        int maxValIdx = 0;
        for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++)
        {
            double contourArea = Imgproc.contourArea(contours.get(contourIdx));
            if (maxVal < contourArea)
            {
                maxVal = contourArea;
                maxValIdx = contourIdx;
            }
        }

        Imgproc.drawContours(input, contours, maxValIdx, new Scalar(255,0,0), 1);
        Rect rect = Imgproc.boundingRect(contours.get(maxValIdx));
        Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(0,255,2), 1);
        int font = Imgproc.FONT_HERSHEY_PLAIN;
        int scale = 1;
        int thickness = 1;
        position = rect.tl();
        String text = rect.tl().toString();
        Imgproc.putText(input, text, position, font, scale, new Scalar(0,255,0), thickness);
        telemetry.addData("the point", position.toString());
        telemetry.update();

        /*
        for (int i = 0; i < contours.size(); i++) {
            Imgproc.rectangle(input, boundRect[i].tl(), boundRect[i].br(), YELLOW, 2);
            int font = Imgproc.FONT_HERSHEY_PLAIN;
            int scale = 1;
            int thickness = 1;
            Point position = boundRect[i].tl();
            String text = boundRect[i].tl().toString();
            Imgproc.putText(input, text, position, font, scale, new Scalar(0,255,0), thickness);
        }
*/

        // Memory cleanup

        return input;
    }
    public Point getCoords() {
        return position;
    }
    // Returns an enum being the current position where the robot will park


}
