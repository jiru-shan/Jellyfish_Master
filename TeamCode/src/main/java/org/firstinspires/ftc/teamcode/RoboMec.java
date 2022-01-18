package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class RoboMec extends LinearOpMode {

    // Anti-tip?  Capstone?
    // Implement lift functions
    // Encoder values for alliance and shared hubs

    public void runOpMode() throws InterruptedException {

        // Motor controllers
        DcMotorController Controller;

        // d - deposit, l - lift, c - carousel
        // left and right with respect to top motors facing right

        // 8 Motors
        DcMotor leftFront;
        DcMotor leftBack;
        DcMotor rightFront;
        DcMotor rightBack;
        DcMotor leftIntake;
        DcMotor rightIntake;
        DcMotor lift_front;
        DcMotor lift_back;

        // 12 Servos
        CRServo carousel;
        Servo d_open;
        Servo d_coverLeft;
        Servo d_coverRight;
        Servo d_bendLeft;
        Servo d_bendRight;
        Servo i_topLeft;
        Servo i_bottomLeft;
        Servo i_topRight;
        Servo i_bottomRight;

        // Color sensors
        // Configuration - REV color/range sensor
        ColorSensor colorSensor_left;
        ColorSensor colorSensor_right;
        int distance = 5;

        // Runtime
        ElapsedTime runtime = new ElapsedTime();

        // Motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        lift_front = hardwareMap.dcMotor.get("lift_front");
        lift_back = hardwareMap.dcMotor.get("lift_back");
        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake = hardwareMap.dcMotor.get("rightIntake");

        // Servos
        d_open = hardwareMap.servo.get("d_open");
        d_coverLeft = hardwareMap.servo.get("d_coverLeft");
        d_coverRight = hardwareMap.servo.get("d_coverRight");
        d_bendLeft = hardwareMap.servo.get("d_bendLeft");
        d_bendRight = hardwareMap.servo.get("d_bendRight");
        i_topLeft = hardwareMap.servo.get("i_topLeft");
        i_bottomLeft = hardwareMap.servo.get("i_bottomLeft");
        i_topRight = hardwareMap.servo.get("i_topRight");
        i_bottomRight = hardwareMap.servo.get("i_bottomRight");
        carousel = hardwareMap.crservo.get("carousel");

        // Color sensors
        colorSensor_left = hardwareMap.get(ColorSensor.class, "colorSensor_left");
        colorSensor_right = hardwareMap.get(ColorSensor.class, "colorSensor_right");

        // Intake
        double highSweepPower = 1.0;
        double lowSweepPower = 0.8;

        // minRange - intake down, deposit position closed, deposit folded down
        // maxRange - intake up, deposit position open, deposit in scoring position

        // Deposit servo positions
        double d_open_minRange = 0.65;
        double d_open_minRangeSemi = 0.63;
        double d_open_top = 0.53;
        double d_open_middle = 0.49;
        double d_open_shared = 0.47;
        double d_minRange_coverLeft = 0.50;
        double d_maxRange_coverLeft = 0.20;
        double d_minRange_coverRight = 0.52;
        double d_maxRange_coverRight = 0.82;
        double d_minRange_bendLeft = 0.89;      // need to fix bend values
        double d_maxRange_bendLeft = 0.78;
        double d_minRange_bendRight = 0.10;
        double d_maxRange_bendRight = 0.21;
        double i_minRange_topLeft = 0.08;
        double i_maxRange_topLeft = 0.85;
        double i_minRange_bottomLeft = 0.92;
        double i_maxRange_bottomLeft = 0.15;
        double i_minRange_topRight = 0.92;
        double i_maxRange_topRight = 0.17;
        double i_minRange_bottomRight = 0.08;
        double i_maxRange_bottomRight = 0.83;

        // Carousel
        double maxSpinPower = 0.5;

        // Factor
        double normalSpeed = 1.0;
        int alliance_targetTipped = 700;
        int alliance_targetBalanced = 625;
        int shared_targetClose = 120;
        int shared_targetMiddle = 200;
        int shared_targetFar = 280;

        // Reset encoders
        lift_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // set motor ticks to 0
        lift_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Initialize deposit to position
        d_open.setPosition(d_open_minRange);
        d_bendLeft.setPosition(d_minRange_bendLeft);
        d_bendRight.setPosition(d_minRange_bendRight);

        waitForStart();

        if (isStopRequested()) return;

        // Run until the end of the match (until press stop on the phone)
        while (opModeIsActive()) {

            // Keep deposit in position while not in use
            d_open.setPosition(d_open_minRange);
            d_bendLeft.setPosition(d_minRange_bendLeft);
            d_bendRight.setPosition(d_minRange_bendRight);
            
//            // intuitive controls
//            double y = gamepad1.right_stick_x; // Reversed
//            double x = -gamepad1.left_stick_x * 1.1; // Strafing + Precision
//            double rx = gamepad1.left_stick_y; // Forward/Backward
//
//            /** Denominator is the largest motor power (absolute value) or 1
//             * This ensures all the powers maintain the same ratio, but only when
//             * at least one is out of the range [-1, 1] **/
//
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            double leftFrontPower = normalSpeed * ((y + x - rx) / denominator);
//            double leftBackPower = normalSpeed * ((y - x - rx) / denominator);
//            double rightFrontPower = normalSpeed * ((y - x + rx) / denominator);
//            double rightBackPower = normalSpeed * ((y + x + rx) / denominator);
//
//            // Transfer calculated power to wheels
//            // Switch "rightFrontPower" and "leftFrontPower" to move joystick right & robot move left
//            // Currently, move joystick right & robot moves right
//            leftFront.setPower(leftBackPower);
//            leftBack.setPower(leftFrontPower);
//            rightFront.setPower(rightFrontPower);
//            rightBack.setPower(rightBackPower);
//
//            // Show the elapsed game time & wheel power
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Motors", "leftFront (%.2f), leftBack (%.2f), rightFront (%.2f), rightBack (%.2f)",
//                    leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
//            telemetry.update();

            // intuitive controls in respect to the back of the robot
            double y = gamepad1.right_stick_x; // Reversed
            double x = gamepad1.left_stick_x * 1.1; // Strafing + Precision
            double rx = -gamepad1.left_stick_y; // Forward/Backward

            /** Denominator is the largest motor power (absolute value) or 1
             * This ensures all the powers maintain the same ratio, but only when
             * at least one is out of the range [-1, 1] **/
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontPower = (y + x - rx) / denominator;
            double leftBackPower = (y - x - rx) / denominator;
            double rightFrontPower = (y - x + rx) / denominator;
            double rightBackPower = (y + x + rx) / denominator;

            leftFront.setPower(leftBackPower);
            leftBack.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);

            // Show the elapsed game time & wheel power
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "leftFront (%.2f), leftBack (%.2f), rightFront (%.2f), rightBack (%.2f)",
                    leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
            telemetry.addData("Color - Left", colorSensor_left.alpha());
            telemetry.addData("Color - Right", colorSensor_right.alpha());
            telemetry.addData("Lift - Front:", lift_front.getCurrentPosition());
            telemetry.addData("Lift - Back:", lift_front.getCurrentPosition());
            telemetry.update();

            // Switch to ninja mode & back
            if (gamepad1.dpad_down) {

                if (normalSpeed == 1.0) {

                    normalSpeed = 0.5;

                } else {

                    normalSpeed = 1.0;
                }
            }

            /** Combined Functions **/

            // Resting position - up
            // With a button, intake should flip down, start intake, check if object present, lift up, ex-take into deposit
            // During this time, one flap of deposit should be up and one should be down

            if (gamepad1.x) {

                // left intake flips down
                i_topLeft.setPosition(i_minRange_topLeft);
                i_bottomLeft.setPosition(i_minRange_bottomLeft);

                // left intake starts running
                leftIntake.setPower(highSweepPower);

                // set deposit to semi-open position
                d_open.setPosition(d_open_minRangeSemi);

                // check if object is present by light/dark values
                if (colorSensor_left.alpha() > 500) {

                    // slow down intake
                    leftIntake.setPower(0.5);

                    // intake lifts up
                    i_topLeft.setPosition(i_maxRange_topLeft);
                    i_bottomLeft.setPosition(i_maxRange_bottomLeft);

                    // stop intake
                    leftIntake.setPower(0);

                    // open left covering of deposit and close right covering of deposit
                    d_coverLeft.setPosition(d_maxRange_coverLeft);
                    d_coverRight.setPosition(d_minRange_coverRight);

                    Thread.sleep(1000);

                    // ex-take object into deposit
                    leftIntake.setPower(-lowSweepPower);

                    Thread.sleep(800);

                    // stop intake
                    leftIntake.setPower(0);

                    // make sure deposit is closed
                    d_open.setPosition(d_open_minRange);

                    // close left covering
                    d_coverLeft.setPosition(d_minRange_coverLeft);
                }
            }

            if (gamepad1.b) {

                // right intake flips down
                i_topRight.setPosition(i_minRange_topRight);
                i_bottomRight.setPosition(i_minRange_bottomRight);

                // right intake starts running
                rightIntake.setPower(highSweepPower);

                // set deposit to semi-open position
                d_open.setPosition(d_open_minRangeSemi);

                // check if object is present by light/dark values
                if (colorSensor_right.alpha() > 500) {

                    // slow down intake
                    rightIntake.setPower(0.5);

                    // intake lifts up
                    i_topRight.setPosition(i_maxRange_topRight);
                    i_bottomRight.setPosition(i_maxRange_bottomRight);

                    // stop intake
                    rightIntake.setPower(0);

                    // open right covering of deposit and close left covering of deposit
                    d_coverRight.setPosition(d_maxRange_coverRight);
                    d_coverLeft.setPosition(d_minRange_coverLeft);

                    Thread.sleep(1000);

                    // ex-take object into deposit
                    rightIntake.setPower(-lowSweepPower);

                    Thread.sleep(800);

                    // stop intake
                    rightIntake.setPower(0);

                    // make sure deposit is closed
                    d_open.setPosition(d_open_minRange);

                    // close right covering
                    d_coverRight.setPosition(d_minRange_coverRight);
                }
            }

            // Intake and Ex-take on Gamepad B

            if (gamepad2.left_bumper) {

                leftIntake.setPower(highSweepPower);

            } else if (gamepad2.right_bumper) {

                rightIntake.setPower(highSweepPower);

            } else if (gamepad2.left_trigger != 0) {

                leftIntake.setPower(-lowSweepPower);

            } else if (gamepad2.right_trigger != 0) {

                rightIntake.setPower(-lowSweepPower);

            } else {

                rightIntake.setPower(0);
                leftIntake.setPower(0);
            }

            /** Lift **/

            // Motor tick count is equal to 384.5

            if (gamepad1.y) {

                // new Compute();

                // Extend arm to deposit position
                lift_front.setTargetPosition(-alliance_targetTipped);
                lift_back.setTargetPosition(-alliance_targetTipped);
                lift_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);   // Move to deposit position
                lift_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift_front.setPower(-1.0);
                lift_back.setPower(-1.0);

                Thread.sleep(1000);

            } else if (gamepad1.dpad_up) {

                // new Compute();

                // Extend arm to deposit position
                lift_front.setTargetPosition(-alliance_targetBalanced);
                lift_back.setTargetPosition(-alliance_targetBalanced);
                lift_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);   // Move to deposit position
                lift_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift_front.setPower(-1.0);
                lift_back.setPower(-1.0);

                Thread.sleep(1000);

            } else if (gamepad1.a) {

                // new Compute();

                // Lift up deposit
                d_bendLeft.setPosition(d_maxRange_bendLeft);
                d_bendRight.setPosition(d_maxRange_bendRight);

                // Open to deposit in top level of alliance hub
                d_open.setPosition(d_open_top);

                Thread.sleep(500);

                // Close & bend down deposit
                d_open.setPosition(d_open_minRange);
                d_bendLeft.setPosition(d_minRange_bendLeft);
                d_bendRight.setPosition(d_minRange_bendRight);

                Thread.sleep(1000);

                // Retract arm to original position
                lift_front.setTargetPosition(0);
                lift_back.setTargetPosition(0);
                lift_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);   // Move to deposit position
                lift_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift_front.setPower(1.0);
                lift_back.setPower(1.0);

                Thread.sleep(1000);

                lift_front.setPower(0);
                lift_back.setPower(0);

            } else if (gamepad2.x) {

                // new Compute();

                lift_front.setTargetPosition(-shared_targetClose);
                lift_back.setTargetPosition(-shared_targetClose);
                lift_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);   // Move to deposit position
                lift_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift_front.setPower(-1.0);
                lift_back.setPower(-1.0);

                Thread.sleep(1000);

            } else if (gamepad2.y) {

                // new Compute();

                lift_front.setTargetPosition(-shared_targetMiddle);
                lift_back.setTargetPosition(-shared_targetMiddle);
                lift_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);   // Move to deposit position
                lift_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift_front.setPower(-1.0);
                lift_back.setPower(-1.0);

                Thread.sleep(1000);

            } else if (gamepad2.b) {

                // new Compute();

                lift_front.setTargetPosition(-shared_targetFar);
                lift_back.setTargetPosition(-shared_targetFar);
                lift_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);   // Move to deposit position
                lift_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift_front.setPower(-1.0);
                lift_back.setPower(-1.0);

                Thread.sleep(1000);

            } else if (gamepad2.a) {

                // Lift up deposit
                d_bendLeft.setPosition(d_maxRange_bendLeft);
                d_bendRight.setPosition(d_maxRange_bendRight);

                // Open to deposit in top level of alliance hub
                d_open.setPosition(d_open_shared);

                Thread.sleep(500);

                // Close & bend down deposit
                d_open.setPosition(d_open_minRange);
                d_bendLeft.setPosition(d_minRange_bendLeft);
                d_bendRight.setPosition(d_minRange_bendRight);

                Thread.sleep(1000);

                lift_front.setTargetPosition(0);
                lift_back.setTargetPosition(0);
                lift_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);   // Move to deposit position
                lift_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift_front.setPower(1.0);
                lift_back.setPower(1.0);

                Thread.sleep(1000);

                lift_front.setPower(0);
                lift_back.setPower(0);
            }

            /** Carousel **/

            // Run Servo
            if (gamepad1.dpad_right) {

                // Spin carousel clockwise
                carousel.setPower(-maxSpinPower);

            } else if (gamepad1.dpad_left) {

                // Spin carousel counterclockwise
                carousel.setPower(maxSpinPower);
            } else {

                // Stop carousel
                carousel.setPower(0);
            }
        }
    }
}
