package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Camera;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.Vision.VisionPipeline;


@Autonomous(name="Vision Test", group="test")
public class VisionTester extends LinearOpMode {

    Camera camera;

    @Override
    public void runOpMode() throws InterruptedException {

        VisionPipeline pipeline = new VisionPipeline();

        camera = new Camera(hardwareMap, pipeline);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {
//            if(pipeline.isGoalVisible()) {
//                telemetry.addData("Rectangle Width", pipeline.goalRect.width);
//                telemetry.addData("Rectangle Height", pipeline.goalRect.height);
//                telemetry.addData("Goal Height (Center line)", pipeline.getGoalHeight());
//              //  telemetry.addData("Goal Height", pipeline.getGoalHeight());
//                //telemetry.addData("Horizontal Distance To Goal", pipeline.getXDistance());
//            } else {
//                telemetry.addLine("Goal not visible");
//            }
//            telemetry.addData("Corners visible",pipeline.isCornersVisible());

            telemetry.addData("Ring Position", pipeline.getRingPosition());
            telemetry.addData("Cb Value", pipeline.getAnalysis());
            telemetry.update();

        }


//        telemetry.addData("Rectangle Cb Value", pipeline.avgCbValue);
//
//        telemetry.addData("Four cb error", pipeline.fourRingCbError);
//        telemetry.addData("Four dimension error", pipeline.fourRingDimensionError);
//        telemetry.addData("Four confidence value", pipeline.fourRingConfidence);
//
//        telemetry.addData("One cb error", pipeline.oneRingCbError);
//        telemetry.addData("One dimension error", pipeline.oneRingDimensionError);
//        telemetry.addData("One confidence value", pipeline.oneRingConfidence);
//
//        telemetry.addData("Hesitance", pipeline.hesitance);
//        telemetry.addData("Ring Position", pipeline.position);
//
//        telemetry.addData("Distance To Ring (inches)", pipeline.distanceToRing);

        telemetry.update();

    }
}
