package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp(name = "Strafe Test", group = "test")
public class StrafeTest extends LinearOpMode {

    Robot robot = new Robot();

    int state = 1;

    static final int STRAFE = 1;
    static final int END_STATE = 2;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initForRunToPosition(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("State", state);
            telemetry.addData("Left Front Motor", robot.chassisLeftFront.getCurrentPosition());
            telemetry.addData("Right Front Motor", robot.chassisRightFront.getCurrentPosition());
            telemetry.addData("Left Back Motor", robot.chassisLeftBack.getCurrentPosition());
            telemetry.addData("Right Back Motor", robot.chassisRightBack.getCurrentPosition());
            telemetry.addData("Robot Ticks Per Inch", robot.robotTicksPerInch);
            telemetry.update();
            switch (state) {
                case STRAFE:
                    if(robot.strafe(0.5, 72)) {
                        goToNextState();
                    }
                    break;

                case END_STATE:
                    robot.stop();
                    break;

                default:
                    break;
            }
        }

    }

    public void goToNextState() {
        state++;
    }
}
