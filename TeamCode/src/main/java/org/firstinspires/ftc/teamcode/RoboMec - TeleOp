package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class RoboMec extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        // Motor controllers
        DcMotorController Controller;

        // Initialize
        DcMotor leftFront;
        DcMotor leftBack;
        DcMotor rightFront;
        DcMotor rightBack;
        DcMotor intake;
        DcMotor armLift;
        DcMotor armRotate;

        // Servo
        CRServo carousel;

        // Setup
        // Controller = hardwareMap.get(DcMotorController.class, "Controller");
        intake = hardwareMap.dcMotor.get("intake");
        carousel = hardwareMap.crservo.get("carousel");
        armLift = hardwareMap.dcMotor.get("armLift");
        armRotate = hardwareMap.dcMotor.get("armRotate");

        // Runtime
        ElapsedTime runtime = new ElapsedTime();

        // Motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        // Carousel
        double minSpinPower = 0;
        double maxSpinPower = 0.5;

        // Factor
        double normalSpeed = 1.0;

        waitForStart();

        if (isStopRequested()) return;

        // Run until the end of the match (until press stop on the phone)
        while (opModeIsActive()) {

            // intuitive controls
            double y = gamepad1.right_stick_x; // Reversed
            double x = -gamepad1.left_stick_x * 1.1; // Strafing + Precision
            double rx = gamepad1.left_stick_y; // Forward/Backward
//
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
            if (gamepad1.a) {

                double lightSweepPower = -0.6;
                // Run intake
                intake.setPower(lightSweepPower);
            } else if (gamepad1.x) {

                double oneSweepPower = -0.8;
                intake.setPower(oneSweepPower);
            } else if (gamepad1.y) {

                double twoSweepPower = -1;
                intake.setPower(twoSweepPower);
            } else if (gamepad1.b) {

                double twoSweepPower = -1;
                intake.setPower(-twoSweepPower);
            } else {                               // If not pressed, stop intake

                double minSweepPower = 0;
                // Stop intake
                intake.setPower(minSweepPower);
            }

            // Intake and Ex-take on Gamepad B

            if (gamepad2.right_bumper) {

                double twoSweepPower = -1;
                intake.setPower(twoSweepPower);
            } else if (gamepad2.left_bumper) {

                double twoSweepPower = -1;
                intake.setPower(-twoSweepPower);
            } else {                                // If not pressed, stop intake

                double minSweepPower = 0;
                // Stop intake
                intake.setPower(minSweepPower);
            }

            /** Arm **/

//            double extend = 720 / 2;
//            double retract = -720/2;
//
//            armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            int reach = armLift.getTargetPosition() + (int) extend;
//
//            if (gamepad2.dpad_up) {
//
//                // Lift arm
//                armLift.setTargetPosition(reach);
//                armLift.setPower(50);
//                armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            } else {
//
//                armLift.setPower(0);
//            }
//
//            // Retract Slides
//            int retractSlides = armLift.getTargetPosition() + (int) retract;
//
//            if (gamepad2.dpad_down) {
//
//                // Lower arm
//                armLift.setTargetPosition(retractSlides);
//                armLift.setPower(-50);
//                armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            } else {
//
//                armLift.setPower(0);
//            }
//
//            // Motor tick count is equal to 28 times gear ratio - 15:1
//            double MOTOR_TICK_COUNT = 420;
//
//            double quarterTurn = (MOTOR_TICK_COUNT / 4);
//            armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // set motor ticks to 0
//
//            // If button "a" on game pad 2 is pressed, run arm
//            if (gamepad2.y) {
//
//                // Move arm to deposit
//                armRotate.setPower(100);
//                armRotate.setTargetPosition((int) quarterTurn);   // Move to deposit position
//                armRotate.setPower(0);
//            }
//
//            // Retract Arm
//            double quarterReverse = (MOTOR_TICK_COUNT / 4);
//
//            if (gamepad2.a) {
//
//                // Move arm to original position
//                armRotate.setPower(-100);
//                armRotate.setTargetPosition((int) quarterReverse);  // Return to original position
//                armRotate.setPower(0);
//            }

            /** Lift & Rotate **/

            double maxLiftPower = 1;
            double minLiftPower = 0;
            double maxRotatePower = 0.3;
            double minRotatePower = 0;

            if (gamepad2.y) {

                // Lift arm
                armLift.setPower(-maxLiftPower);
                // Rotate forward
                armRotate.setPower(maxRotatePower);
            } else if (gamepad2.a) {

                // Lower arm
                armLift.setPower(maxLiftPower);
                // Rotate backward
                armRotate.setPower(-maxRotatePower);
            } else if (gamepad2.dpad_up){

                // Rotate forward
                armRotate.setPower(maxRotatePower);
            } else if (gamepad2.dpad_down){

                // Rotate backward
                armRotate.setPower(-maxRotatePower);
            } else {

                // Stop arm
                armLift.setPower(minLiftPower);
                // Stop deposit
                armRotate.setPower(minRotatePower);
            }

            /** Lift **/
//
//            double maxLiftPower = 1;
//            double minLiftPower = 0;
//
//            if (gamepad2.dpad_down) {
//
//                // Lift arm
//                armLift.setPower(maxLiftPower);
//            } else if (gamepad2.dpad_up) {
//
//                // Lower arm
//                armLift.setPower(-maxLiftPower);
//            } else {
//
//                // Stop arm
//                armLift.setPower(minLiftPower);
//            }
//
            /** Rotate **/
//
//            double maxRotatePower = 0.3;
//            double minRotatePower = 0;
//
//            if (gamepad2.y) {
//
//                // Rotate forward
//                armRotate.setPower(maxRotatePower);
//            } else if (gamepad2.a) {
//
//                // Rotate backward
//                armRotate.setPower(-maxRotatePower);
//            } else {
//
//                // Stop
//                armRotate.setPower(minRotatePower);
//            }

            /** Carousel **/

            // Run Servo
            if (gamepad2.b) {

                // Spin carousel clockwise
                carousel.setPower(-maxSpinPower);

            } else if (gamepad2.x) {

                // Spin                          carousel counterclockwise
                carousel.setPower(maxSpinPower);
            } else {

                // Stop carousel
                carousel.setPower(minSpinPower);

            }

            /** Sensors **/

                // If there is one cube on the ramp/in the deposit, then stop intake
        }
    }
}
