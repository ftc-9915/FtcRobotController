/* Uses contouring to actively identify and draw a box around the largest ring.
Then uses the Cb channel value of the area to check for orange color and
the aspect ratio of the drawn box to build a "confidence value" from 0 to 1 for ring positions
FOUR and ONE and a hesitance value for ring position NONE. These confidence values
are then used to determine the most accurate Ring Position.

 */



package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.BlueGoalVisionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


/**
 * Vision Pipeline that uses contours to draw a rectangle around the largest orange object and uses the aspect ratio of the rectangle and cb color channel values of the contents of the
 * rectangle to determine confidence ratings for the one, four and none configuration, and determines the most likely ring configuration
 *
 */
@Config
public class BlueGoalVisionPipeline extends OpenCvPipeline {

    public class Fraction {
        private int numerator, denominator;
        Fraction(long a, long b) {
            numerator = (int) (a / gcd(a, b));
            denominator = (int) (b / gcd(a, b));
        }
        /**
         * @return the greatest common denominator
         */
        private long gcd(long a, long b) {
            return b == 0 ? a : gcd(b, a % b);
        }
        public int getNumerator() {
            return numerator;
        }
        public int getDenominator() {
            return denominator;
        }
    }


    //Pinhole Camera Variables
    public static final double CAMERA_HEIGHT = 8.25; //camera height inches for distance calculation
    public static final double HIGH_GOAL_HEIGHT = 35.875; //camera height inches for distance calculation

    private int imageWidth;
    private int imageHeight;

    private double cameraPitchOffset;
    private double cameraYawOffset;

    private double fov;
    private double imageArea;
    private double offsetCenterX;
    private double offsetCenterY;
    private double horizontalFocalLength;
    private double verticalFocalLength;


    //Boundary Line (Only detects above this to eliminate field tape)
    public static final int BOUNDARY = 160;


    //Mask constants to isolate blue coloured subjects
    public static double lowerH = 106;
    public static double lowerS = 78;
    public static double lowerV = 135;

    public static double upperH = 118;
    public static double upperS = 255;
    public static double upperV = 255;

    public Scalar lowerHSV = new Scalar(lowerH, lowerS, lowerV);
    public Scalar upperHSV = new Scalar(upperH, upperS, upperV);

    //working mat variables
    public Mat HSVFrame = new Mat();
    public Mat MaskFrame = new Mat();
    public Mat ContourFrame = new Mat();


    //USEFUL VALUES TO BE ACCESSED FROM AUTONOMOUS
    public Rect goalRect = new Rect();

    public int viewfinderIndex = 0;

    @Override
    public void init(Mat firstFrame) {
        imageWidth = firstFrame.width();
        imageHeight = firstFrame.height();

        imageArea = this.imageWidth * this.imageHeight;
        offsetCenterX = ((double) this.imageWidth / 2) - 0.5;
        offsetCenterY = ((double) this.imageHeight / 2) - 0.5;

        // pinhole model calculations
        double diagonalView = Math.toRadians(this.fov);
        Fraction aspectFraction = new Fraction(this.imageWidth, this.imageHeight);
        int horizontalRatio = aspectFraction.getNumerator();
        int verticalRatio = aspectFraction.getDenominator();
        double diagonalAspect = Math.hypot(horizontalRatio, verticalRatio);
        double horizontalView = Math.atan(Math.tan(diagonalView / 2) * (horizontalRatio / diagonalAspect)) * 2;
        double verticalView = Math.atan(Math.tan(diagonalView / 2) * (verticalRatio / diagonalAspect)) * 2;
        horizontalFocalLength = this.imageWidth / (2 * Math.tan(horizontalView / 2));
        verticalFocalLength = this.imageHeight / (2 * Math.tan(verticalView / 2));
    }



    @Override
    public Mat processFrame(Mat input) {


        //define largestAreaRect in method so it resets every cycle
        double largestAreaRect = 0;

        //Convert to HSV color space
        Imgproc.cvtColor(input, HSVFrame, Imgproc.COLOR_RGB2HSV);

        //Mask Frame to only include tuned blue objects
        Core.inRange(HSVFrame, lowerHSV, upperHSV, MaskFrame);

        //Erode excess items in mask
        Imgproc.erode(MaskFrame, MaskFrame, new Mat(), new Point(-1, -1), 1);

        //create arraylist to populate with contour points
        ArrayList<MatOfPoint> contourList = new ArrayList<>();

        //find contours, outputs points to "contours", outputs image to "ContourFrame"
        Imgproc.findContours(MaskFrame, contourList, ContourFrame, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE); //RETR_Tree retrieves contour with layered hierarchy

        //traverse through contourList to find contour with largest width, which is presumed to be ring
        for(MatOfPoint contour : contourList){

            MatOfPoint2f convertedContour = new MatOfPoint2f(contour.toArray());

            //draw rectangle around contour
            Rect contourRect = Imgproc.boundingRect(convertedContour);

            //find largest area contour rectangle in list that's above boundary line
            if(contourRect.y + contourRect.height < BOUNDARY  &&
                    contourRect.area() > largestAreaRect) {
                largestAreaRect = contourRect.area();
                goalRect = contourRect;
            }

            contour.release(); // releasing the buffer of the contour, since after use, it is no longer needed
            convertedContour.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
        }
        //draw Rect
        Imgproc.rectangle(input, goalRect, new Scalar(0,255,0), 2);

        //Return MaskFrame for tuning purposes
//        return MaskFrame;
        if(viewfinderIndex % 2 == 0){
            return input;
        } else {
            return MaskFrame;
        }
    }

    //helper method to check if rect is found
    public boolean isGoalVisible(){
        return goalRect != null;
    }

    public double getGoalPitch() {
        if(goalRect == null){
            return -1;
        }
        double targetCenterY = goalRect.y + goalRect.height / 2;
        return -Math.toDegrees(
                Math.atan((offsetCenterY - targetCenterY) / verticalFocalLength));
    }
    public double getGoalYaw() {
        if(goalRect == null){
            return -1;
        }

        double targetCenterX = goalRect.x + goalRect.width / 2;
        return Math.toDegrees(
                Math.atan((offsetCenterX - targetCenterX) / horizontalFocalLength));
    }

    public double getGoalDistance(){
        double xDistance = getXDistance();
        double yDistance = HIGH_GOAL_HEIGHT - CAMERA_HEIGHT;
        double diagonalDistance = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
        return diagonalDistance;
    }

    public double getXDistance(){
        return (5642.0/goalRect.width) - 0.281;
    }

    @Override
    public void onViewportTapped() {
        /*
         * Changing the displayed color space on the viewport when the user taps it
         */

        viewfinderIndex++;
    }





}
