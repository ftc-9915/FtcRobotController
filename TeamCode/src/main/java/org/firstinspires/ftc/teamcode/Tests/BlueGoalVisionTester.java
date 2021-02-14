package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Vision.Camera;
import org.firstinspires.ftc.teamcode.Vision.BlueGoalVisionPipeline;


@Autonomous(name="Blue Goal Vision Test", group="test")
public class BlueGoalVisionTester extends LinearOpMode {

    Camera camera;
    @Override
    public void runOpMode() throws InterruptedException {

        BlueGoalVisionPipeline blueVision = new BlueGoalVisionPipeline(telemetry);
        camera = new Camera(hardwareMap, blueVision);


        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            if(gamepad1.a){
                blueVision.onViewportTapped();
            }

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
