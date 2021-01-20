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



    //Pinhole Camera Variables
    public static final double CAMERA_HEIGHT = 8.25; //camera height inches for distance calculation
    public static final double HIGH_GOAL_HEIGHT = 35.875; //camera height inches for distance calculation


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


    //Variables for distance calculation

    public Rect goalRect = new Rect();

    Point upperLeftCorner;
    Point upperRightCorner;
    Point lowerLeftCorner;
    Point lowerRightCorner;

    Point upperMiddle;
    Point lowerMiddle;


    public int viewfinderIndex = 0;

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
                findCorners(contour.toArray());
            }

            contour.release(); // releasing the buffer of the contour, since after use, it is no longer needed
            convertedContour.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
        }

        //draw Rect
//        Imgproc.rectangle(input, goalRect, new Scalar(0,255,0), 2);

        //draw corners
        Imgproc.circle(input, upperLeftCorner,2, new Scalar(0,255,0),5);
        Imgproc.circle(input, upperRightCorner,2, new Scalar(0,255,0),5);
        Imgproc.circle(input, lowerLeftCorner,2, new Scalar(0,255,0),5);
        Imgproc.circle(input, lowerRightCorner,2, new Scalar(0,255,0),5);

        Imgproc.circle(input, upperMiddle,2, new Scalar(255,0,0),2);
        Imgproc.circle(input, lowerMiddle,2, new Scalar(255,0,0),2);

        Imgproc.line(input, upperMiddle, lowerMiddle, new Scalar(255, 0, 0));
        Imgproc.putText(input, "Goal Height: " + Math.round(getGoalHeight()) + "px",  new Point(upperMiddle.x, (upperMiddle.y + lowerMiddle.y) / 2), 1, 0.5, new Scalar(255, 255, 255));
        Imgproc.putText(input, "Distance from robot: " + Math.round(getXDistance()) + "in",  new Point(upperMiddle.x, (upperMiddle.y + lowerMiddle.y) / 2 + 10), 1, 0.5, new Scalar(255, 255, 255));


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


    public double getXDistance(){
        return (5642.0/getGoalHeight()) - 0.281;
    }

    //"Real Height" gives returns an accurate goal height in pixels even when goal is viewed at an angle by calculating distance between middle two points
    public double getGoalHeight(){
        return Math.sqrt(Math.pow(lowerMiddle.y - upperMiddle.y, 2) + Math.pow(Math.abs(lowerMiddle.x - upperMiddle.x), 2));
    }

    @Override
    public void onViewportTapped() {
        /*
         * Changing the displayed color space on the viewport when the user taps it
         */

        viewfinderIndex++;
    }

    public void findCorners(Point[] goalContourPoints){

        //Default all points to first point in list
        upperLeftCorner = upperRightCorner = lowerLeftCorner = lowerRightCorner = goalContourPoints[0];

        //Find corner points
        for (Point point : goalContourPoints){
            double x = point.x;
            double y = point.y;

            //Check if point is upper leftmost point
            if((-x + y) > (-upperLeftCorner.x + upperLeftCorner.y)){
                upperLeftCorner = point;
                //Check if point is upper rightmost point
            } else if ((x + y) > (upperRightCorner.x + upperRightCorner.y)){
                upperRightCorner = point;
                //Check if point is lower leftmost point
            } else if ((-x - y) > (-lowerLeftCorner.x - lowerRightCorner.y)) {
                lowerLeftCorner = point;
            }
            //Check if point is lower rightmost point
            else if((x - y) > (lowerRightCorner.x - lowerRightCorner.y)){
                lowerRightCorner = point;
            }
        }

        //Calculate "middle points" for true height calculation
        upperMiddle = new Point((upperLeftCorner.x + upperRightCorner.x) / 2, (upperLeftCorner.y + upperRightCorner.y) / 2);
        lowerMiddle = new Point((lowerLeftCorner.x + lowerRightCorner.x) / 2, (lowerLeftCorner.y + lowerRightCorner.y) / 2);

    }





}



