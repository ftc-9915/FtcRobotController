package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class VisionPipeline extends OpenCvPipeline {
    // -- TWEAKABLE CONSTANTS ---

    public boolean viewportPaused = false;

    //ringAnalysisZone location, b2 (60, 140)
    public static final Point ZONE_CENTER = new Point(53, 120);
    public static final int ZONE_WIDTH = 45;
    public static final int ZONE_HEIGHT = 50;

    //Cb Threshhold Values
    private static final int FOUR_RING_THRESHOLD = 149;
    private static final int ONE_RING_THRESHOLD = 128;

    //Viewfinder Colorspace
    public ViewfinderType[] COLOR_SPACES = new ViewfinderType[] {ViewfinderType.RGB, ViewfinderType.YcrCb, ViewfinderType.Cb};
    public int viewfinderIndex = 0;

    //Generated Submat Rectangle Points
    public Point zoneUpperLeft = new Point( ZONE_CENTER.x - ZONE_WIDTH/2 ,ZONE_CENTER.y - ZONE_HEIGHT/2 );
    public Point zoneLowerRight = new Point( ZONE_CENTER.x + ZONE_WIDTH/2 ,ZONE_CENTER.y + ZONE_HEIGHT/2 );


    // -- WOKRING VARIABLES --
    private Mat YcrCbFrame = new Mat();
    private Mat CbFrame = new Mat();
    private Mat ringAnalysisZone = new Mat();

    private int avgCbValue;
    public volatile RingPosition position;

    /**
     * An enum to define the ring position
     */
    public enum RingPosition
    {
        FOUR,
        ONE,
        NONE
    }

    /**
     * An enum to define the viewfinder colorspace
     */
    public enum ViewfinderType{
        RGB,
        YcrCb,
        Cb
    }

    /**
     *
     * @param  -  Matrix of pixels representing raw image from first frame
     * Sets ring analysis zone by setting submat and extracting Cb channel
     * from first frame
     */
    @Override
    public void init(Mat firstFrame) {


        //extract Cb color space
        Imgproc.cvtColor(firstFrame, YcrCbFrame, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YcrCbFrame, CbFrame, 1);

        //create persistent reference for submat, any changes to CbFrame will change ringAnalysisZone
        ringAnalysisZone = CbFrame.submat(new Rect(zoneUpperLeft, zoneLowerRight));
    }



    /**
     * Converts and extracts Cb channel from submat and determines
     * RingPosition based on threshold values.
     * @param input -  Matrix of pixels representing raw image
     * @return input displayed on viewport with colorspace defined by COLOR_SPACE
     */
    @Override
    public Mat processFrame(Mat input) {
        //extract Cb color space
        Imgproc.cvtColor(input, YcrCbFrame, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YcrCbFrame, CbFrame, 1);

        //find average cb value of the submat
        avgCbValue = (int) Core.mean(ringAnalysisZone).val[0];


        // Set position based on Cb values
        if(avgCbValue > FOUR_RING_THRESHOLD){
            position = RingPosition.FOUR;
        }else if (avgCbValue > ONE_RING_THRESHOLD){
            position = RingPosition.ONE;
        }else {
            position = RingPosition.NONE;
        }

        //switch between rendered color spaces
        switch (COLOR_SPACES[viewfinderIndex % COLOR_SPACES.length]){
            case YcrCb:
                input = YcrCbFrame;
            case Cb:
                input = CbFrame;
            default:

        }
        Imgproc.rectangle(input, zoneUpperLeft, zoneLowerRight, new Scalar(0, 0, 255), 2);
        return input;
    }

    @Override
    public void onViewportTapped() {
        /*
         * Changing the displayed color space on the viewport when the user taps it
         */

        viewfinderIndex++;
    }
    public int getAnalysis(){
        return avgCbValue;
    }


}