package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class RoboMecAsync extends LinearOpMode {

    enum State {}

    // Motor controllers
    DcMotorController Controller;

    // d - deposit, l - lift, c - carousel
    // left and right with respect to top motors facing right

    // Left and right intake switched in configuration

    // 8 Motors
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor leftIntake;
    DcMotor rightIntake;
    DcMotor liftFront;
    DcMotor liftBack;

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
    ColorRangeSensor colorSensor_left;
    ColorRangeSensor colorSensor_right;

    // Deposit servo positions
    double d_open_minRange = 0.65;
    double d_open_minRangeSemi = 0.63;
    double d_open_top = 0.53;
    double d_open_middle = 0.49;   // Fix this value
    double d_open_shared = 0.47;
    double d_minRange_coverLeft = 0.55;
    double d_maxRange_coverLeft = 0.15;
    double d_minRange_coverRight = 0.45;
    double d_maxRange_coverRight = 0.85;
    double d_minRange_bendLeft = 0.96;
    double d_maxRange_bendLeft = 0.78;
    double d_minRange_bendRight = 0.05;
    double d_maxRange_bendRight = 0.23;

    double d_maxRange_bendLeft_middle = 0.10;   // Fix these values
    double d_maxRange_bendRight_middle = 0.21;

    double i_minRange_topLeft = 0.10;   // 0.08
    double i_maxRange_topLeft = 0.88;
    double i_minRange_bottomLeft = 0.91;   // 0.92
    double i_maxRange_bottomLeft = 0.12;
    double i_minRange_topRight = 0.96;   // 0.94
    double i_maxRange_topRight = 0.20;
    double i_minRange_bottomRight = 0.04;   // 0.08
    double i_maxRange_bottomRight = 0.80;

    // Carousel
    double maxSpinPower = 0.5;

    // Factor
    double normalSpeed = 1.0;
    int alliance_targetTipped = 550;
    int alliance_targetBalanced = 525;   // Fix this value
    int alliance_middle = 580;   // Fix this value
    int shared_targetClose = 120;
    int shared_targetMiddle = 200;
    int shared_targetFar = 280;

    LiftAsync lift;
    AsyncMacroHandler gei;
    // Runtime
    ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        // Motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        liftFront = hardwareMap.dcMotor.get("lift_front");
        liftBack = hardwareMap.dcMotor.get("lift_back");
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
        colorSensor_left = hardwareMap.get(ColorRangeSensor.class, "colorSensor_left");
        colorSensor_right = hardwareMap.get(ColorRangeSensor.class, "colorSensor_right");

        gei = new AsyncMacroHandler();
        // Intake
        double highSweepPower = 0.8;
        // double lowSweepPower = 0.4;

        // minRange - intake down, deposit position closed, deposit folded down
        // maxRange - intake up, deposit position open, deposit in scoring position


        // Reset encoders
        liftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // set motor ticks to 0
        liftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        d_open.setPosition(d_open_minRangeSemi);
        d_bendLeft.setPosition(d_minRange_bendLeft);
        d_bendRight.setPosition(d_minRange_bendRight);

        d_coverLeft.setPosition(d_minRange_coverLeft);
        d_coverRight.setPosition(d_minRange_coverRight);

        waitForStart();

        if (isStopRequested()) return;

        // Run until the end of the match (until press stop on the phone)
        while (opModeIsActive()) {

            // Keep deposit in position while not in use
            d_open.setPosition(d_open_minRange);
            d_bendLeft.setPosition(d_minRange_bendLeft);
            d_bendRight.setPosition(d_minRange_bendRight);

            // intuitive controls in respect to the back of the robot
            double y = -gamepad1.right_stick_x; // Reversed
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
            telemetry.addData("Lift - Front:", liftFront.getCurrentPosition());
            telemetry.addData("Lift - Back:", liftBack.getCurrentPosition());
            telemetry.update();


            if (!gei.isBusy()) {
                if (gamepad1.left_bumper) {
                    gei.setState(AsyncMacroHandler.MacroState.gamepad1_lb);
                } else if (gamepad1.right_bumper) {
                    gei.setState(AsyncMacroHandler.MacroState.gamepad1_rb);
                } else if (gamepad2.dpad_up) {
                    gei.setState(AsyncMacroHandler.MacroState.gamepad2_dpu);
                } else if (gamepad2.dpad_right) {
                    gei.setState(AsyncMacroHandler.MacroState.gamepad2_dpr);
                } else if (gamepad2.dpad_down) {
                    gei.setState(AsyncMacroHandler.MacroState.gamepad2_dpd);
                } else if (gamepad2.dpad_left) {
                    gei.setState(AsyncMacroHandler.MacroState.gamepad2_dpl);
                } else if (gamepad2.x) {
                    gei.setState(AsyncMacroHandler.MacroState.gamepad2_x);
                } else if (gamepad2.y) {
                    gei.setState(AsyncMacroHandler.MacroState.gamepad2_y);
                } else if (gamepad2.b) {
                    gei.setState(AsyncMacroHandler.MacroState.gamepad2_b);
                } else if (gamepad2.a) {
                    gei.setState(AsyncMacroHandler.MacroState.gamepad2_a);
                }
            }

            /** Carousel **/

            // Run Servo
            if (gamepad2.right_bumper) {

                // Spin carousel clockwise
                carousel.setPower(-maxSpinPower);

            } else if (gamepad2.left_bumper) {

                // Spin carousel counterclockwise
                carousel.setPower(maxSpinPower);

            } else {

                // Stop carousel
                carousel.setPower(0);
            }
            gei.execute();


        }
    }
}