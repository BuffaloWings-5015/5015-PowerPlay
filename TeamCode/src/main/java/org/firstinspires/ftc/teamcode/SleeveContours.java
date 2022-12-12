package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SleeveContours extends OpenCvPipeline {
    /*
    YELLOW  = Parking Left
    CYAN    = Parking Middle
    MAGENTA = Parking Right
     */
    Telemetry telemetry;


    // TOPLEFT anchor point for the bounding box
    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(445, 78);

    // Width and height for the bounding box
    public static int REGION_WIDTH = 300;
    public static int REGION_HEIGHT = 500;


    // Lower and upper boundaries for colors
    private static final Scalar
            lower_yellow_bounds  = new Scalar(20, 120, 120),
            magentaLower = new Scalar(140, 100, 100),
            magentaUpper = new Scalar(160, 255, 255),
            cyaentaLower= new Scalar(90,100,100),
            cyaentaUpper = new Scalar(140,255,255),
            upper_yellow_bounds  = new Scalar(30, 255, 255);


    // Color definitions
    private final Scalar
            YELLOW  = new Scalar(150,255,255),
            CYAN    = new Scalar(0, 255, 255);


    // Percent and mat definitions
    private double yelPercent, cyaPercent, magPercent;
    private Mat yelMat = new Mat(),
            mat = new Mat(),
            cyaMat = new Mat(),
            magMat = new Mat(),
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

    public SleeveContours(Telemetry telemetry) {
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

        Imgproc.drawContours(input, contours, maxValIdx, new Scalar(255,255,0), 1);
        Rect rect = Imgproc.boundingRect(contours.get(maxValIdx));
        Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(255,255,0), 1);
        int font = Imgproc.FONT_HERSHEY_PLAIN;
        int scale = 1;
        int thickness = 1;
        Point position = rect.tl();
        String text = rect.tl().toString();
        Imgproc.putText(input, text, position, font, scale, new Scalar(255,255,0), thickness);
        // MAGENTA ##################################################################################################

        Core.inRange(blurredMat, magentaLower, magentaUpper, magMat);
        List<MatOfPoint> magcontours = new ArrayList<>();
        Mat maghierarchey = new Mat();
        Imgproc.findContours(magMat, magcontours, maghierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        //Drawing the Contours
        MatOfPoint2f[] magcontoursPoly  = new MatOfPoint2f[magcontours.size()];
        Rect[] magboundRect = new Rect[magcontours.size()];
        Point[] magcenters = new Point[magcontours.size()];
        float[][] magradius = new float[magcontours.size()][1];
        for (int i = 0; i < magcontours.size(); i++) {
            magcontoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(magcontours.get(i).toArray()), magcontoursPoly[i], 3, true);
            magboundRect[i] = Imgproc.boundingRect(new MatOfPoint(magcontoursPoly[i].toArray()));
            magcenters[i] = new Point();
            Imgproc.minEnclosingCircle(magcontoursPoly[i], magcenters[i], magradius[i]);

            //Adding text to the image
            /*
            Imgproc.putText(input, text, position, font, scale, new Scalar(90,245,255), thickness);

             */
        }
        List<MatOfPoint> magcontoursPolyList = new ArrayList<>(magcontoursPoly.length);
        for (MatOfPoint2f poly : magcontoursPoly) {
            magcontoursPolyList.add(new MatOfPoint(poly.toArray()));
        }

        double magmaxVal = 0;
        int magmaxValIdx = 0;
        for (int magcontourIdx = 0; magcontourIdx < contours.size(); magcontourIdx++)
        {
            double magcontourArea = Imgproc.contourArea(contours.get(magcontourIdx));
            if (magmaxVal < magcontourArea)
            {
                magmaxVal = magcontourArea;
                magmaxValIdx = magcontourIdx;
            }
        }

        Imgproc.drawContours(input, magcontours, magmaxValIdx, new Scalar(255,0,255), 1);
        Rect magrect = Imgproc.boundingRect(magcontours.get(magmaxValIdx));
        Imgproc.rectangle(input, magrect.tl(), magrect.br(), new Scalar(255,0,255), 1);
        Point magposition = magrect.tl();
        String magtext = magrect.tl().toString();
        Imgproc.putText(input, magtext, magposition, font, scale, new Scalar(255,0,255), thickness);

        // Memory cleanup

        // cyaENTA ##################################################################################################

        Core.inRange(blurredMat, cyaentaLower, cyaentaUpper, cyaMat);
        List<MatOfPoint> cyacontours = new ArrayList<>();
        Mat cyahierarchey = new Mat();
        Imgproc.findContours(cyaMat, cyacontours, cyahierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        //Drawing the Contours
        MatOfPoint2f[] cyacontoursPoly  = new MatOfPoint2f[cyacontours.size()];
        Rect[] cyaboundRect = new Rect[cyacontours.size()];
        Point[] cyacenters = new Point[cyacontours.size()];
        float[][] cyaradius = new float[cyacontours.size()][1];
        for (int i = 0; i < cyacontours.size(); i++) {
            cyacontoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(cyacontours.get(i).toArray()), cyacontoursPoly[i], 3, true);
            cyaboundRect[i] = Imgproc.boundingRect(new MatOfPoint(cyacontoursPoly[i].toArray()));
            cyacenters[i] = new Point();
            Imgproc.minEnclosingCircle(cyacontoursPoly[i], cyacenters[i], cyaradius[i]);

            //Adding text to the icyae
            /*
            Imgproc.putText(input, text, position, font, scale, new Scalar(90,245,255), thickness);

             */
        }
        List<MatOfPoint> cyacontoursPolyList = new ArrayList<>(cyacontoursPoly.length);
        for (MatOfPoint2f poly : cyacontoursPoly) {
            cyacontoursPolyList.add(new MatOfPoint(poly.toArray()));
        }

        double cyamaxVal = 0;
        int cyamaxValIdx = 0;
        for (int cyacontourIdx = 0; cyacontourIdx < contours.size(); cyacontourIdx++)
        {
            double cyacontourArea = Imgproc.contourArea(contours.get(cyacontourIdx));
            if (cyamaxVal < cyacontourArea)
            {
                cyamaxVal = cyacontourArea;
                cyamaxValIdx = cyacontourIdx;
            }
        }
        Imgproc.drawContours(input, cyacontours, cyamaxValIdx, new Scalar(0,255,255), 1);
        Rect cyarect = Imgproc.boundingRect(cyacontours.get(cyamaxValIdx));
        Imgproc.rectangle(input, cyarect.tl(), cyarect.br(), new Scalar(0,255,255), 1);
        Point cyaposition = cyarect.tl();
        String cyatext = cyarect.tl().toString();
        Imgproc.putText(input, cyatext, cyaposition, font, scale, new Scalar(0,255,255), thickness);
        return input;
    }

    // Returns an enum being the current position where the robot will park


}
