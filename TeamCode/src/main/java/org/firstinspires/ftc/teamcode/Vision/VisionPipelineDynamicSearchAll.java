package org.firstinspires.ftc.teamcode.Vision;

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
import java.util.Collections;
import java.util.Comparator;



public class VisionPipelineDynamicSearchAll extends OpenCvPipeline {

    /**
     * An enum to define the ring position
     */
    public enum RingPosition
    {
        FOUR,
        ONE,
        NONE
    }



    //Cb Threshhold Values and Aspect Ratios (width/height)
    private static final double FOUR_RING_CB_AVERAGE = 153.667;
    private static final double FOUR_RING_ASPECT_RATIO = 1.117405583;

    private static final double ONE_RING_CB_AVERAGE = 148;
    private static final double ONE_RING_ASPECT_RATIO = 1.878156566;

    private static final double ERROR_BEFORE_NO_RING = 0.2;

    public static final double DIMENSION_WEIGHT = 1;
    public static final double COLOR_WEIGHT = 1;

    //definitions for upper and lower threshholds for orange mask in the YcrCb channel mode
    private Scalar lowerOrange = new Scalar(0.0, 141.0, 0.0);
    private Scalar upperOrange = new Scalar(255.0, 230.0, 95.0);

    //working variables
    private Mat YcrCbFrame = new Mat();
    private Mat MaskFrame = new Mat();
    private Mat ContourFrame = new Mat();
    private Mat RingAnalysisZone = new Mat();
    private Mat CbFrame = new Mat();

    private Rect ringAnalysisRect;

    private int avgCbValue;
    public volatile RingPosition position;


    @Override
    public Mat processFrame(Mat input) {
        //define maxWidth in method so it resets every cycle
        int maxWidth = 0;

        //extract Cb color space
        Imgproc.cvtColor(input, YcrCbFrame, Imgproc.COLOR_RGB2YCrCb);

        //apply mask to isolate orange rings
        Core.inRange(YcrCbFrame, lowerOrange, upperOrange, MaskFrame);

        //apply blur to remove extraneous results
        Imgproc.GaussianBlur(MaskFrame, MaskFrame, new Size(5.0, 15.0), 0.00);

        //create arraylist to populate with contour points
        ArrayList<MatOfPoint> contourList = new ArrayList<>();

        //create empty list to sort rectangles
        ArrayList<Rect> contourRectangles = new ArrayList<Rect>();

        //find contours, outputs points to "contours", outputs image to "ContourFrame"
        Imgproc.findContours(MaskFrame, contourList, ContourFrame, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE); //RETR_Tree retrieves contour with layered hierarchy

        //traverse through contourList to find contour with largest width, which is presumed to be ring
        for(MatOfPoint contour : contourList){

            MatOfPoint2f convertedContour = new MatOfPoint2f(contour.toArray());

            //create rectangle around contour
            Rect contourRect = Imgproc.boundingRect(convertedContour);

            //add to list
            contourRectangles.add(contourRect);

            contour.release(); // releasing the buffer of the contour, since after use, it is no longer needed
            convertedContour.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
        }

        //sort from greatest to least
        RectangleComparator comparator = new RectangleComparator();
        Collections.sort(contourRectangles, comparator);

        position = RingPosition.NONE;
        ringAnalysisRect = new Rect();
        for(Rect rect : contourRectangles) {
            double fourRingDimensionError = Math.abs(FOUR_RING_ASPECT_RATIO  - ((double) rect.width / rect.height));
            double oneRingDimensionError = Math.abs(ONE_RING_ASPECT_RATIO  - ((double) rect.width / rect.height));
            if(fourRingDimensionError < 0.15 || oneRingDimensionError < 0.15) {
                ringAnalysisRect = analyzeRect(rect, fourRingDimensionError, oneRingDimensionError);
                break;
            }
        }
        Imgproc.rectangle(input, ringAnalysisRect, new Scalar(0,255,0), 2);
        Imgproc.putText(input, String.valueOf(position) , new Point(ringAnalysisRect.x, ringAnalysisRect.y - 20), Imgproc.FONT_HERSHEY_PLAIN, 0.5, new Scalar(0,255,0), 1);

        return input;


    }

    public Rect analyzeRect(Rect rect, double fourRingDimensionError, double oneRingDimensionError){
        Core.extractChannel(YcrCbFrame, CbFrame, 1);
        RingAnalysisZone = CbFrame.submat(rect);

        //find average cb value of the submat
        avgCbValue = (int) Core.mean(RingAnalysisZone).val[0];

        double fourRingCbError = (double) Math.abs(FOUR_RING_CB_AVERAGE - avgCbValue) / FOUR_RING_CB_AVERAGE;
        double fourRingConfidence = 1 - ((fourRingCbError * COLOR_WEIGHT) + (fourRingDimensionError * DIMENSION_WEIGHT));

        double oneRingCbError = (double) Math.abs(ONE_RING_CB_AVERAGE - avgCbValue) / ONE_RING_CB_AVERAGE;
        double oneRingConfidence = 1 - ((oneRingCbError * COLOR_WEIGHT) + (oneRingDimensionError * DIMENSION_WEIGHT));

        if (oneRingConfidence > fourRingConfidence){
            position = RingPosition.ONE;
        } else {
            position = RingPosition.FOUR;
        }

        return rect;
    }


}

class RectangleComparator implements Comparator<Rect> {

    @Override
    public int compare(Rect r1, Rect r2) {
        return r2.width - r1.width;
    }
}
