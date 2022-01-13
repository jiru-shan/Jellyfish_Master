package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class RoboMec extends LinearOpMode {

    // Testing
    // Missing 1 Servo

    public void runOpMode() throws InterruptedException {

        // Motor controllers
        DcMotorController Controller;

        // d - deposit, l - lift, c - carousel

        // 9 Motors
        DcMotor leftFront;
        DcMotor leftBack;
        DcMotor rightFront;
        DcMotor rightBack;
        DcMotor leftIntake;   // left and right with respect to top motors facing right
        DcMotor rightIntake;
        DcMotor lift_1;
        DcMotor lift_2;

        // 12 Servos
        CRServo carousel;
        Servo d_open;
        Servo d_coverLeft;
        Servo d_coverRight;
        Servo d_bendLeft;
        Servo d_bendRight;
        Servo i_right_1;
        Servo i_right_2;
        Servo i_left_1;
        Servo i_left_2;

        // Runtime
        ElapsedTime runtime = new ElapsedTime();

        // Motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        lift_1 = hardwareMap.dcMotor.get("lift_1");
        lift_2 = hardwareMap.dcMotor.get("lift_2");
        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake = hardwareMap.dcMotor.get("rightIntake");

        // Servos
        d_open = hardwareMap.servo.get("d_open");
        d_coverLeft = hardwareMap.servo.get("d_coverLeft");
        d_coverRight = hardwareMap.servo.get("d_coverRight");
        d_bendLeft = hardwareMap.servo.get("d_bendLeft");
        d_bendRight = hardwareMap.servo.get("d_bendRight");
        i_left_1 = hardwareMap.servo.get("i_left_1");
        i_left_2 = hardwareMap.servo.get("i_left_2");
        i_right_1 = hardwareMap.servo.get("i_right_1");
        i_right_2 = hardwareMap.servo.get("i_right_2");
        carousel = hardwareMap.crservo.get("carousel");

        // Intake
        double twoSweepPower = -1;
        double minSweepPower = 0;

        // Deposit
        double d_open_minRange = 0;
        double d_open_maxRange = 0.3;
        double d_cover_minRange = 0;
        double d_cover_maxRange = 0.5;
        double d_bend_minRange = 0;
        double d_bend_maxRange = 0.6;
        double i_minRange = 0;
        double i_maxRange = 0.3;

        // Carousel
        double minSpinPower = 0;
        double maxSpinPower = 0.5;

        // Factor
        double normalSpeed = 1.0;
        int target = 100;

        // Reset encoders
        lift_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // set motor ticks to 0
        lift_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // PID
        double fineTune;
        double position;
        double goal;

        waitForStart();

        if (isStopRequested()) return;

        // Run until the end of the match (until press stop on the phone)
        while (opModeIsActive()) {

            // intuitive controls
            double y = gamepad1.right_stick_x; // Reversed
            double x = -gamepad1.left_stick_x * 1.1; // Strafing + Precision
            double rx = gamepad1.left_stick_y; // Forward/Backward

//            /** Denominator is the largest motor power (absolute value) or 1
//             * This ensures all the powers maintain the same ratio, but only when
//             * at least one is out of the range [-1, 1] **/

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontPower = normalSpeed * ((y + x - rx) / denominator);
            double leftBackPower = normalSpeed * ((y - x - rx) / denominator);
            double rightFrontPower = normalSpeed * ((y - x + rx) / denominator);
            double rightBackPower = normalSpeed * ((y + x + rx) / denominator);

            // Transfer calculated power to wheels
            // Switch "rightFrontPower" and "leftFrontPower" to move joystick right & robot move left
            // Currently, move joystick right & robot moves right
            leftFront.setPower(leftBackPower);
            leftBack.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);
//
//            // Show the elapsed game time & wheel power
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "leftFront (%.2f), leftBack (%.2f), rightFront (%.2f), rightBack (%.2f)",
                    leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
            telemetry.update();

//            // intuitive controls in respect to the back of the robot
//            double y = gamepad1.right_stick_x; // Reversed
//            double x = gamepad1.left_stick_x * 1.1; // Strafing + Precision
//            double rx = -gamepad1.left_stick_y; // Forward/Backward
//
//            /** Denominator is the largest motor power (absolute value) or 1
//             * This ensures all the powers maintain the same ratio, but only when
//             * at least one is out of the range [-1, 1] **/
//            double denominator = Math.max(Math.abs(y)   + Math.abs(x) + Math.abs(rx), 1);
//            double leftFrontPower = (y + x - rx) / denominator;
//            double leftBackPower = (y - x - rx) / denominator;
//            double rightFrontPower = (y - x + rx) / denominator;
//            double rightBackPower = (y + x + rx) / denominator;
//
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

            // Switch to ninja mode & back
            if (gamepad1.right_bumper) {

                if (normalSpeed == 1.0) {

                    normalSpeed = 0.5;

                } else {

                    normalSpeed = 1.0;
                }
            }

            /** Intake **/

            // If button "a, x, y, b" on game pad 1 is pressed, run intake
            if (gamepad1.left_bumper) {

                leftIntake.setPower(twoSweepPower);

            } else if (gamepad1.right_bumper) {

                rightIntake.setPower(twoSweepPower);

            } else if (gamepad1.left_trigger != 0) {

                leftIntake.setPower(-twoSweepPower);

            } else if (gamepad1.right_trigger != 0) {

                rightIntake.setPower(-twoSweepPower);

            } else if (gamepad1.x) {

                // Raise
                i_left_1.setPosition(i_maxRange);
                i_left_2.setPosition(i_maxRange);
                i_right_1.setPosition(i_maxRange);
                i_right_2.setPosition(i_maxRange);

                i_right_1.setPosition(i_maxRange);

            } else if (gamepad1.dpad_up && gamepad1.x) {

                i_left_1.setPosition(i_minRange);
                i_left_2.setPosition(i_minRange);
                i_right_1.setPosition(i_minRange);
                i_right_2.setPosition(i_minRange);

            } else {                               // If not pressed, stop intake

                leftIntake.setPower(minSweepPower);
                rightIntake.setPower(minSweepPower);
                i_left_1.setPosition(i_minRange);
                i_left_2.setPosition(i_minRange);
                i_right_1.setPosition(i_minRange);
                i_right_2.setPosition(i_minRange);
            }

            // Intake and Ex-take on Gamepad B

            if (gamepad2.left_bumper) {

                leftIntake.setPower(twoSweepPower);

            } else if (gamepad2.right_bumper) {

                rightIntake.setPower(-twoSweepPower);

            } else if (gamepad2.left_trigger != 0) {

                leftIntake.setPower(-twoSweepPower);

            } else if (gamepad2.right_trigger != 0) {

                rightIntake.setPower(-twoSweepPower);

            } else {

                rightIntake.setPower(minSweepPower);
                leftIntake.setPower(minSweepPower);
            }

            /** Lift **/

            // Motor tick count is equal to 28 times gear ratio - 15:1

// If button "a" on game pad 2 is pressed, run arm
            if (gamepad2.a) {

                // Move arm to deposit
                lift_1.setPower(0.3);
                lift_2.setPower(0.3);
                lift_1.setTargetPosition(target);
                lift_2.setTargetPosition(target);
                lift_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);   // Move to deposit position
                lift_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            } else if (gamepad2.dpad_up && gamepad2.a) {

                // Move arm to original position
                lift_1.setPower(0.3);
                lift_2.setPower(0.3);
                lift_1.setTargetPosition(0);
                lift_2.setTargetPosition(0);
                lift_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);   // Move to deposit position
                lift_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            } else {

                lift_1.setPower(0);
                lift_2.setPower(0);
            }

            /** Deposit **/

            if (gamepad2.x) {

                // Open
                d_open.setPosition(d_open_maxRange);

            } else if (gamepad2.dpad_up && gamepad2.x) {

                // Close
                d_open.setPosition(d_open_minRange);

            } else if (gamepad2.b) {

                // Open
                d_coverLeft.setPosition(d_cover_maxRange);
                d_coverRight.setPosition(d_cover_maxRange);

            } else if (gamepad2.dpad_up && gamepad2.b) {

                // Cover
                d_coverLeft.setPosition(d_cover_minRange);
                d_coverRight.setPosition(d_cover_minRange);

            } else if (gamepad2.y) {

                // Up
                d_bendLeft.setPosition(d_bend_maxRange);
                d_bendRight.setPosition(d_bend_maxRange);

            } else if (gamepad2.dpad_up && gamepad2.y) {

                // Down
                d_bendLeft.setPosition(d_bend_minRange);
                d_bendRight.setPosition(d_bend_minRange);

            } else {

                d_open.setPosition(d_open_minRange);
                d_coverLeft.setPosition(d_cover_minRange);
                d_coverRight.setPosition(d_cover_minRange);
                d_bendLeft.setPosition(d_bend_minRange);
                d_bendRight.setPosition(d_bend_minRange);
            }

            /** Carousel **/

            // Run Servo
            if (gamepad2.dpad_right) {

                // Spin carousel clockwise
                carousel.setPower(-maxSpinPower);

            } else if (gamepad2.dpad_left) {

                // Spin carousel counterclockwise
                carousel.setPower(maxSpinPower);
            } else {

                // Stop carousel
                carousel.setPower(minSpinPower);
            }

            // PID


            /** Sensors **/

            // If there is one cube on the ramp/in the deposit, then stop intake
        }
    }
}
