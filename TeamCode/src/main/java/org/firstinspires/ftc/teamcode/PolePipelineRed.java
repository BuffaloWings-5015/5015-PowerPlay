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

public class PolePipelineRed extends OpenCvPipeline {
    /*
    YELLOW  = Parking Left
    redN    = Parking Middle
    blueENTA = Parking Right
     */
    Telemetry telemetry;
    public Point position;
    public Point redPosition;
    public Point bluePosition;
    public int x;
    int font = Imgproc.FONT_HERSHEY_PLAIN;
    int scale = 5;
    int thickness = 5;
    public enum PolePosition {
        LEFT,
        CENTER,
        RIGHT,
    }
    public enum PoleWidth {
        FAR,
        CENTER,
        CLOSE,
    }
    public PolePosition polePositoin = PolePosition.CENTER ;
    public PoleWidth widthofpole = PoleWidth.CENTER;
    // TOPLEFT anchor point for the bounding box
    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(445, 78);

    // Width and height for the bounding box
    public static int REGION_WIDTH = 300;
    public static int REGION_HEIGHT = 500;


    // Lower and upper boundaries for colors
    private static final Scalar
            lower_yellow_bounds  = new Scalar(20, 50, 50),
            upper_yellow_bounds  = new Scalar(50, 255, 255),
            lower_blue_bounds = new Scalar(100, 50, 50),
            upper_blue_bounds = new Scalar(130, 255, 255),
            lower_red_bounds = new Scalar(0,50,50),
            upper_red_bounds = new Scalar(10,255,255);


    // Color definitions
    private final Scalar
            YELLOW  = new Scalar(150,255,255),
            redN    = new Scalar(0, 255, 255);


    // Percent and mat definitions
    private double yelPercent, redPercent, bluePercent;
    private Mat yelMat = new Mat(),
            mat = new Mat(),
            yelYelMat = new Mat(),
            blurredMat = new Mat(),
            blueMat = new Mat(),
            redMat = new Mat(),
            kernel = new Mat();

    // Anchor point definitions
    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Running variable storing the parking position

    public PolePipelineRed(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    @Override
    public Mat processFrame(Mat input) {
        // Noise reduction
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Imgproc.blur(mat, blurredMat, new Size(5, 5));

        // Apply Morphology
        /*
        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel);

        // Gets channels from given source mat
        Core.inRange(blurredMat, lower_yellow_bounds, upper_yellow_bounds, yelMat);
        if (Core.countNonZero(yelMat) > 0) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchey = new Mat();
        Imgproc.findContours(yelMat, contours, hierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        //Drawing the Contours
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        Point[] centers = new Point[contours.size()];
        float[][] radius = new float[contours.size()][1];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            centers[i] = new Point();
            Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i]);

            //Adding text to the ibluee

            Imgproc.putText(input, text, position, font, scale, new Scalar(90,245,255), thickness);


        }
        List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
        for (MatOfPoint2f poly : contoursPoly) {
            contoursPolyList.add(new MatOfPoint(poly.toArray()));
        }

        double maxVal = 0;
        int maxValIdx = 0;
        for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++) {
            double contourArea = Imgproc.contourArea(contours.get(contourIdx));
            if (maxVal < contourArea) {
                maxVal = contourArea;
                maxValIdx = contourIdx;
            }
        }
        */

            Core.inRange(blurredMat, lower_blue_bounds, upper_blue_bounds, blueMat);
        if (Core.countNonZero(blueMat) > 0){
            List<MatOfPoint> bluecontours = new ArrayList<>();
            Mat bluehierarchey = new Mat();
            Imgproc.findContours(blueMat, bluecontours, bluehierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            //Drawing the Contours
            MatOfPoint2f[] bluecontoursPoly = new MatOfPoint2f[bluecontours.size()];
            Rect[] blueboundRect = new Rect[bluecontours.size()];
            Point[] bluecenters = new Point[bluecontours.size()];
            float[][] blueradius = new float[bluecontours.size()][1];
            for (int i = 0; i < bluecontours.size(); i++) {
                bluecontoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(bluecontours.get(i).toArray()), bluecontoursPoly[i], 3, true);
                blueboundRect[i] = Imgproc.boundingRect(new MatOfPoint(bluecontoursPoly[i].toArray()));
                bluecenters[i] = new Point();
                Imgproc.minEnclosingCircle(bluecontoursPoly[i], bluecenters[i], blueradius[i]);

                //Adding text to the ibluee


            }
            List<MatOfPoint> bluecontoursPolyList = new ArrayList<>(bluecontoursPoly.length);
            for (MatOfPoint2f poly : bluecontoursPoly) {
                bluecontoursPolyList.add(new MatOfPoint(poly.toArray()));
            }

            double bluemaxVal = 0;
            int bluemaxValIdx = 0;
            for (int bluecontourIdx = 0; bluecontourIdx < bluecontours.size(); bluecontourIdx++) {
                double bluecontourArea = Imgproc.contourArea(bluecontours.get(bluecontourIdx));
                if (bluemaxVal < bluecontourArea) {
                    bluemaxVal = bluecontourArea;
                    bluemaxValIdx = bluecontourIdx;
                }
            }
            Rect blueRect = Imgproc.boundingRect(bluecontours.get(bluemaxValIdx));
            Imgproc.rectangle(input, blueRect.tl(), blueRect.br(), new Scalar(0, 0, 255), 1);
            bluePosition = blueRect.tl();
            String bluetext = blueRect.tl().toString();
            Imgproc.putText(input, bluetext, bluePosition, font, scale, new Scalar(0, 0, 255), thickness);
        }

            Core.inRange(blurredMat, lower_red_bounds, upper_red_bounds, redMat);
        if (Core.countNonZero(redMat) > 0){
            List<MatOfPoint> redcontours = new ArrayList<>();
            Mat redhierarchey = new Mat();
            Imgproc.findContours(redMat, redcontours, redhierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            //Drawing the Contours
            MatOfPoint2f[] redcontoursPoly = new MatOfPoint2f[redcontours.size()];
            Rect[] redboundRect = new Rect[redcontours.size()];
            Point[] redcenters = new Point[redcontours.size()];
            float[][] redradius = new float[redcontours.size()][1];
            for (int i = 0; i < redcontours.size(); i++) {
                redcontoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(redcontours.get(i).toArray()), redcontoursPoly[i], 3, true);
                redboundRect[i] = Imgproc.boundingRect(new MatOfPoint(redcontoursPoly[i].toArray()));
                redcenters[i] = new Point();
                Imgproc.minEnclosingCircle(redcontoursPoly[i], redcenters[i], redradius[i]);

                //Adding text to the irede
            /*
            Imgproc.putText(input, text, position, font, scale, new Scalar(90,245,255), thickness);

             */
            }
            List<MatOfPoint> redcontoursPolyList = new ArrayList<>(redcontoursPoly.length);
            for (MatOfPoint2f poly : redcontoursPoly) {
                redcontoursPolyList.add(new MatOfPoint(poly.toArray()));
            }

            double redmaxVal = 0;
            int redmaxValIdx = 0;
            for (int redcontourIdx = 0; redcontourIdx < redcontours.size(); redcontourIdx++) {
                double redcontourArea = Imgproc.contourArea(redcontours.get(redcontourIdx));
                if (redmaxVal < redcontourArea) {
                    redmaxVal = redcontourArea;
                    redmaxValIdx = redcontourIdx;
                }
            }
            Rect redRect = Imgproc.boundingRect(redcontours.get(redmaxValIdx));
            Imgproc.rectangle(input, redRect.tl(), redRect.br(), new Scalar(255, 0, 0), 1);
            redPosition = redRect.tl();
            String redtext = redRect.tl().toString();
            Imgproc.putText(input, redtext, redPosition, font, scale, new Scalar(255, 0, 0), thickness);
        }
            /*
        Imgproc.drawContours(input, contours, maxValIdx, new Scalar(255, 0, 0), 1);
        Rect rect = Imgproc.boundingRect(contours.get(maxValIdx));
        */







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
        mat.release();
        yelMat.release();
        yelYelMat.release();
        blurredMat.release();
        kernel.release();
        return input;
    }
    public PolePosition getCoords() {
        return polePositoin;
    }
    public PoleWidth getWidth() {
        return widthofpole;
    }
    // Returns an enum being the current position where the robot will park


}
