package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Collector Test", group="test")
public class CollectorTest extends OpMode {

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;



    DcMotor launcherMotor;

    DcMotor collectorMotor;

    DcMotor wobbleMotor;

    Servo liftServo;
    Servo pushServo;
    Servo wobbleServo;

    //Servo feederServo;

    // TODO: test these and edit with accurate values
    static final double SERVO_RESET_POS = 1.0;
    static final double SERVO_PUSH_POS = 0.0;

    double launcherPower;
    double collectorPower;
    boolean collectorPush;
    int wobbleArmPos;

    // Ensures that the adjustments are made each time the gamepad buttons are pressed rather than each time through loop
    boolean buttonReleased1;
    boolean buttonReleased2;

    static final double LIFT_UP_POS = 0.49 ;
    static final double LIFT_DOWN_POS = 0.75;
    static final double NOT_PUSH_POS = 0.68;
    static final double PUSH_POS = 0.52;

    // TODO: test these and edit with accurate values
    static final int WOBBLE_ARM_SPEED = 10;
    static final int WOBBLE_ARM_UPPER_LIMIT = 1000;
    static final int WOBBLE_ARM_LOWER_LIMIT = 0;
    static final double WOBBLE_CLAW_OPEN = 1.0;
    static final double WOBBLE_CLAW_CLOSE = 0.0;


    double speed = 0.0;
    double strafe = 0.0;
    double rotation = 0.0;

    @Override
    public void init() {
        // Chassis Motors

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);


        // Attachment Motors
        launcherMotor = hardwareMap.dcMotor.get("launcherMotor");
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectorMotor = hardwareMap.dcMotor.get("collectorMotor");
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftServo = hardwareMap.servo.get("liftServo");
        pushServo = hardwareMap.servo.get("pushServo");

        /*
        wobbleMotor = hardwareMap.dcMotor.get("wobbleMotor");
        wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleMotor.setTargetPosition(0);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleServo = hardwareMap.servo.get("wobbleServo");
         */

        //feederServo = hardwareMap.servo.get("feederServo");
        //feederServo.setPosition(SERVO_RESET_POS);

        // Initialization values
        launcherPower = 0.0;
        collectorPower = 1.0;
        collectorPush = false;
        wobbleArmPos = 0;
        buttonReleased1 = true;
        buttonReleased2 = true;

        // Starting position
        liftServo.setPosition(LIFT_DOWN_POS);
        pushServo.setPosition(NOT_PUSH_POS);

    }

    @Override
    public void loop() {
        // Controls and Information
        telemetry.addLine("--- Controls (Gamepad 1) ---");
        telemetry.addData("Turn collector on", "Button A");
        telemetry.addData("Turn collector off", "Button B");
        telemetry.addData("Reverse collector direction", "Button X");
        telemetry.addLine();
        telemetry.addLine("--- Controls (Gamepad 2) ---");
        telemetry.addData("Turn launcher on/off", "Button X");
        telemetry.addData("Push/retract collector servo", "Button Y");
        telemetry.addData("Lower collector platform", "Left Bumper");
        telemetry.addData("Lift collector platform", "Right Bumper");

        // Chassis code
        speed = -gamepad1.right_stick_y;
        strafe = gamepad1.right_stick_x;
        rotation = gamepad1.left_stick_x;


        leftFront.setPower(speed + strafe + rotation);
        leftBack.setPower(speed - strafe + rotation);
        rightBack.setPower(speed + strafe - rotation);
        rightFront.setPower(speed - strafe - rotation);



        // Turns collector on/off
        if (gamepad1.a && buttonReleased1) {
            collectorPower = 1.0;
            buttonReleased1 = false;
        }

        if (gamepad1.b && buttonReleased1) {
            collectorPower = 0.0;
            buttonReleased1 = false;
        }

        // Reverses collector

        if (gamepad1.x && buttonReleased1) {
            collectorPower *= -1;
            buttonReleased1 = false;
        }

        // Turns launcher on/off
        if (gamepad2.x && buttonReleased2) {
            if (launcherPower == 0.0) {
                launcherPower = -1.0;
            } else {
                launcherPower = 0.0;
            }
            buttonReleased2 = false;
        }

        // Pushes/retracts collector servo
        if (gamepad2.y && buttonReleased2) {
            if (collectorPush) {
                pushServo.setPosition(NOT_PUSH_POS);
                collectorPush = false;
            } else {
                pushServo.setPosition(PUSH_POS);
                collectorPush = true;
            }
            buttonReleased2 = false;
        }

        // Lifts/Lowers the collecting platform
        if (gamepad2.left_bumper && buttonReleased2) {
            liftServo.setPosition(LIFT_DOWN_POS);
            buttonReleased2 = false;
        }

        if (gamepad2.right_bumper && buttonReleased2) {
            liftServo.setPosition(LIFT_UP_POS);
            buttonReleased2 = false;
        }

        // Lifts/Lowers Wobble Goal Arm
        /*
        wobbleArmPos += WOBBLE_ARM_SPEED * gamepad2.left_stick_y;
        if (wobbleArmPos > WOBBLE_ARM_UPPER_LIMIT) {
            wobbleArmPos = WOBBLE_ARM_UPPER_LIMIT;
        }
        if (wobbleArmPos < WOBBLE_ARM_LOWER_LIMIT) {
            wobbleArmPos = WOBBLE_ARM_LOWER_LIMIT;
        }
        wobbleMotor.setTargetPosition(wobbleArmPos);
        wobbleMotor.setPower(1.0);
         */

        // Opens/Closes Wobble Goal Claw
        /*
        if (gamepad2.a && buttonReleased2) {
            wobbleServo.setPosition(WOBBLE_CLAW_OPEN);
            buttonReleased2 = false;
        }
        if (gamepad2.b && buttonReleased2) {
            wobbleServo.setPosition(WOBBLE_CLAW_CLOSED);
            buttonReleased2 = false;
        }
         */


        // Do not adjust values again until after buttons are released (and pressed again) so the
        // adjustments are made each time the gamepad buttons are pressed rather than each time through loop
        if (!gamepad1.a && !gamepad1.b && !gamepad1.x) {
            buttonReleased1 = true;
        }

        if(!gamepad2.left_bumper && !gamepad2.right_bumper && !gamepad2.a && !gamepad2.b && !gamepad2.x && !gamepad2.y) {
            buttonReleased2 = true;
        }

        launcherMotor.setPower(launcherPower);
        collectorMotor.setPower(collectorPower);

    }
}
