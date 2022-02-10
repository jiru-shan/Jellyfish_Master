package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@TeleOp
public class RoboRed extends LinearOpMode {

    // Left and right intake switched in configuration

    // 8 Motors
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor leftIntake;
    DcMotor rightIntake;
    DcMotor liftLeft;
    DcMotor liftRight;

    // 12 Servos
    Servo i_topLeft;
    Servo i_bottomLeft;
    Servo i_topRight;
    Servo i_bottomRight;
    CRServo c_Left;
    CRServo c_Right;
    Servo bucket;
    Servo arm;


    // Color sensors
    ColorRangeSensor colorSensor_left;
    ColorRangeSensor colorSensor_right;


    // Runtime
    ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        // Motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        liftLeft = hardwareMap.dcMotor.get("liftLeft");
        liftRight = hardwareMap.dcMotor.get("liftRight");
        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake = hardwareMap.dcMotor.get("rightIntake");

        // Servos
        i_topLeft = hardwareMap.servo.get("i_topLeft");
        i_bottomLeft = hardwareMap.servo.get("i_bottomLeft");
        i_topRight = hardwareMap.servo.get("i_topRight");
        i_bottomRight = hardwareMap.servo.get("i_bottomRight");
        bucket = hardwareMap.servo.get("bucket");
        arm = hardwareMap.servo.get("arm");
        c_Left = hardwareMap.crservo.get("c_Left");
        c_Right = hardwareMap.crservo.get("c_Right");

        // Color sensors
        colorSensor_left = hardwareMap.get(ColorRangeSensor.class, "colorSensor_left");
        colorSensor_right = hardwareMap.get(ColorRangeSensor.class, "colorSensor_right");

        // Intake
        double highSweepPower = 0.8;
        // double lowSweepPower = 0.4;

        // Deposit servo positions
        double bucket_down = 0.55;
        double bucket_left = 0.27;
        double bucket_right = 0.83;
        double arm_forward = 0.30;   // temporary
        double arm_backward = 0.87;
        double turret_center = 0.5;

        double i_minRange_topLeft = 0.15;   // 0.08
        double i_maxRange_topLeft = 0.87;
        double i_minRange_bottomLeft = 0.90;   // 0.92
        double i_maxRange_bottomLeft = 0.18;
        double i_minRange_topRight = 0.87;   // 0.94
        double i_maxRange_topRight = 0.16;
        double i_minRange_bottomRight = 0.16;   // 0.08
        double i_maxRange_bottomRight = 0.87;

        // Carousel
        double maxSpinPower = 0.5;

        // Factor
        double normalSpeed = 1.0;
        int alliance_targetTipped = 625;
        int alliance_targetBalanced = 575;   // Fix this value
        int alliance_middle = 550;   // Fix this value
        int shared_targetClose = 120;
        int shared_targetMiddle = 200;
        int shared_targetFar = 280;

        // Reset encoders
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // set motor ticks to 0
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean objectCaptured = false;

        int leftIntakeState = 0;
        ElapsedTime leftIntakeTime = new ElapsedTime();
        int rightIntakeState = 0;
        ElapsedTime rightIntakeTime = new ElapsedTime();

        double intakeCaptureDistance = 6.0;
        double intakeEjectDistance = 30.0;
        int intakeFlipUpTime = 250;
        /*
        0 = not moving
        1 = flip down + intaking
        2 = detected cube + flip up + open flap
        3 = transfering
        4 = transfered + flap closed
        */
        int liftState = 0;
        int liftPosition = 0;
        int liftExtendError = 100;
        int liftRetractError = 10;
        ElapsedTime liftTime = new ElapsedTime();
        /*
        0 = down
        12 = shared close
        13 = shared mid
        14 = shared far
        15 = alliance mid
        16 = alliance bal
        17 = aliance tip
        20-? = retracting
        ? = retracted
        30-? manual control
        */
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();

        if (isStopRequested()) return;

        // Run until the end of the match (until press stop on the phone)
        while (opModeIsActive()) {

            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            //inits
            // Keep deposit in position while not in use
            // d_open.setPosition(d_open_minRangeSemi);
            // d_bendLeft.setPosition(d_minRange_bendLeft);
            // d_bendRight.setPosition(d_minRange_bendRight);


            /** Combined Functions **/

            // Resting position - up
            // With a button, intake should flip down, start intake, check if object present, lift up, ex-take into deposit
            // During this time, one flap of deposit should be up and one should be down


            if (leftIntakeState == 0) {

                if (gamepad1.left_bumper) {

                    i_topLeft.setPosition(i_minRange_topLeft);
                    i_bottomLeft.setPosition(i_minRange_bottomLeft);

                    leftIntake.setPower(highSweepPower);

                    bucket.setPosition(bucket_down);

                    leftIntakeTime.reset();

                    leftIntakeState++;

                } else {
                    i_topLeft.setPosition(i_maxRange_topLeft);
                    i_bottomLeft.setPosition(i_maxRange_bottomLeft);
                }

            } else if (((rightIntakeState > 0 && rightIntakeState <= 9) || (gamepad1.left_bumper && leftIntakeTime.milliseconds() > 400)) && (leftIntakeState < 9)) {
                leftIntakeState = 10;

            } else if (leftIntakeState == 1) {

                if (colorSensor_left.getDistance(DistanceUnit.CM) < intakeCaptureDistance) {
                    leftIntake.setPower(0.25);

                    i_topLeft.setPosition(i_maxRange_topLeft);
                    i_bottomLeft.setPosition(i_maxRange_bottomLeft);

                    leftIntakeState++;

                } else {
                    i_topLeft.setPosition(i_minRange_topLeft);
                    i_bottomLeft.setPosition(i_minRange_bottomLeft);
                }

            } else if (leftIntakeState == 2) {

                leftIntake.setPower(0);

                bucket.setPosition(bucket_left);

                leftIntakeTime.reset();

                leftIntakeState++;

            } else if (leftIntakeState == 3) {

                if (leftIntakeTime.milliseconds() > intakeFlipUpTime) {
                    leftIntake.setPower(-highSweepPower);

                    leftIntakeTime.reset();

                    leftIntakeState++;
                }

            } else if (leftIntakeState == 10) {

                // leftIntake.setPower(-1);
                leftIntakeTime.reset();
                leftIntakeState++;

            } else if (leftIntakeState == 11) {

                // if (leftIntakeTime.milliseconds() > 50) {
                leftIntake.setPower(0);

                i_topLeft.setPosition(i_maxRange_topLeft);
                i_bottomLeft.setPosition(i_maxRange_bottomLeft);

                bucket.setPosition(bucket_down);

                leftIntakeState = 0;
                // }
            } else {

                leftIntake.setPower(0);

                i_topLeft.setPosition(i_maxRange_topLeft);
                i_bottomLeft.setPosition(i_maxRange_bottomLeft);

                // d_open.setPosition(d_open_minRangeSemi);

                bucket.setPosition(bucket_down);

                leftIntakeState = 0;
            }

            if (rightIntakeState == 0) {

                if (gamepad1.right_bumper) {

                    i_topRight.setPosition(i_minRange_topRight);
                    i_bottomRight.setPosition(i_minRange_bottomRight);

                    rightIntake.setPower(highSweepPower);

                    bucket.setPosition(bucket_down);

                    rightIntakeTime.reset();

                    rightIntakeState++;

                } else {

                    i_topRight.setPosition(i_maxRange_topRight);
                    i_bottomRight.setPosition(i_maxRange_bottomRight);
                }

            } else if (((leftIntakeState > 0 && leftIntakeState <= 9) || (gamepad1.right_bumper && rightIntakeTime.milliseconds() > 400)) && (rightIntakeState < 9)) {

                rightIntakeState = 10;

            } else if (rightIntakeState == 1) {

                if (colorSensor_right.getDistance(DistanceUnit.CM) < intakeCaptureDistance) {
                    rightIntake.setPower(0.25);

                    i_topRight.setPosition(i_maxRange_topRight);
                    i_bottomRight.setPosition(i_maxRange_bottomRight);

                    rightIntakeState++;

                } else {

                    i_topRight.setPosition(i_minRange_topRight);
                    i_bottomRight.setPosition(i_minRange_bottomRight);
                }

            } else if (rightIntakeState == 2) {

                rightIntake.setPower(0);

                bucket.setPosition(bucket_right);

                rightIntakeTime.reset();

                rightIntakeState++;

            } else if (rightIntakeState == 3) {

                if (rightIntakeTime.milliseconds() > intakeFlipUpTime) {
                    rightIntake.setPower(-highSweepPower);

                    rightIntakeTime.reset();

                    rightIntakeState++;
                }

            } else if (rightIntakeState == 10) {

                // rightIntake.setPower(-1);
                rightIntakeTime.reset();
                rightIntakeState++;

            } else if (rightIntakeState == 11) {

                // if (rightIntakeTime.milliseconds() > 50) {
                rightIntake.setPower(0);

                i_topRight.setPosition(i_maxRange_topRight);
                i_bottomRight.setPosition(i_maxRange_bottomRight);

                bucket.setPosition(bucket_down);

                rightIntakeState = 0;
                // }

            } else {

                rightIntake.setPower(0);

                i_topRight.setPosition(i_maxRange_topRight);
                i_bottomRight.setPosition(i_maxRange_bottomRight);

                // d_open.setPosition(d_open_minRangeSemi);

                bucket.setPosition(bucket_down);

                rightIntakeState = 0;

            }

            /** Lift **/

            // Motor tick count is equal to 384.5
            if (liftState == 0) {

                if (gamepad2.a) {

                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftLeft.setTargetPosition(-shared_targetClose);
                    liftRight.setTargetPosition(-shared_targetClose);
                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    bucket.setPosition(bucket_down);
                    liftLeft.setPower(-1.0);
                    liftRight.setPower(-1.0);
                    liftState = 12;
                    liftPosition = 1;

                } else if (gamepad2.b) {

                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftLeft.setTargetPosition(-shared_targetMiddle);
                    liftRight.setTargetPosition(-shared_targetMiddle);
                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    bucket.setPosition(bucket_down);
                    liftLeft.setPower(-1.0);
                    liftRight.setPower(-1.0);
                    liftState = 13;
                    liftPosition = 1;

                } else if (gamepad2.y) {

                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftLeft.setTargetPosition(-shared_targetFar);
                    liftRight.setTargetPosition(-shared_targetFar);
                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    bucket.setPosition(bucket_down);
                    liftLeft.setPower(-1.0);
                    liftRight.setPower(-1.0);
                    liftState = 14;
                    liftPosition = 1;

                } else if (gamepad2.dpad_down) {

                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftLeft.setTargetPosition(-alliance_middle);
                    liftRight.setTargetPosition(-alliance_middle);
                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    bucket.setPosition(bucket_down);
                    liftLeft.setPower(-1.0);
                    liftRight.setPower(-1.0);
                    liftState = 15;
                    liftPosition = 1;

                } else if (gamepad2.dpad_left) {

                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftLeft.setTargetPosition(-alliance_targetBalanced);
                    liftRight.setTargetPosition(-alliance_targetBalanced);
                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    bucket.setPosition(bucket_down);
                    liftLeft.setPower(-1.0);
                    liftRight.setPower(-1.0);
                    liftState = 16;
                    liftPosition = 1;

                } else if (gamepad2.dpad_up) {

                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftLeft.setTargetPosition(-alliance_targetTipped);
                    liftRight.setTargetPosition(-alliance_targetTipped);
                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    bucket.setPosition(bucket_down);
                    liftLeft.setPower(-1.0);
                    liftRight.setPower(-1.0);
                    liftState = 17;
                    liftPosition = 1;
                }

            } else if (gamepad2.left_trigger != 0 && liftLeft.getTargetPosition() < 500) {

                bucket.setPosition(bucket_down);
                liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftLeft.setTargetPosition(800);
                liftRight.setTargetPosition(800);
                liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftLeft.setPower(1.0);
                liftRight.setPower(1.0);
                liftState = 0;

            } else if (gamepad2.x && liftState < 20) {

                if (Math.abs(liftLeft.getCurrentPosition()) > 100) {

                    arm.setPosition(arm_forward);
                    bucket.setPosition(bucket_right);

                    liftState = 21;
                    liftTime.reset();
                }

            } else if (gamepad2.dpad_right && liftState < 20) {

                if (Math.abs(liftLeft.getCurrentPosition()) > 100) {

                    arm.setPosition(arm_forward);
                    bucket.setPosition(bucket_left);

                    liftState = 21;
                    liftTime.reset();
                }

            } else if (liftState >= 10 && liftState < 20) {

                if (gamepad2.a && (liftLeft.getTargetPosition() != (-shared_targetClose))) {

                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftLeft.setTargetPosition(-shared_targetClose);
                    liftRight.setTargetPosition(-shared_targetClose);
                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if (liftLeft.getCurrentPosition() < (-shared_targetClose)) {

                        liftLeft.setPower(1.0);
                        liftRight.setPower(1.0);

                    } else {

                        liftLeft.setPower(-1.0);
                        liftRight.setPower(-1.0);
                    }

                    liftState = 12;
                    liftPosition = 1;

                } else if (gamepad2.b && (liftLeft.getTargetPosition() != (-shared_targetMiddle))) {

                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftLeft.setTargetPosition(-shared_targetMiddle);
                    liftRight.setTargetPosition(-shared_targetMiddle);
                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLeft.setPower(-1.0);
                    liftRight.setPower(-1.0);

                    if (liftLeft.getCurrentPosition() < (-shared_targetMiddle)) {

                        liftLeft.setPower(1.0);
                        liftRight.setPower(1.0);

                    } else {

                        liftLeft.setPower(-1.0);
                        liftRight.setPower(-1.0);
                    }

                    liftState = 13;
                    liftPosition = 1;

                } else if (gamepad2.y && (liftLeft.getTargetPosition() != (-shared_targetFar))) {

                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftLeft.setTargetPosition(-shared_targetFar);
                    liftRight.setTargetPosition(-shared_targetFar);
                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLeft.setPower(-1.0);
                    liftRight.setPower(-1.0);

                    if (liftLeft.getCurrentPosition() < (-shared_targetFar)) {

                        liftLeft.setPower(1.0);
                        liftRight.setPower(1.0);

                    } else {

                        liftLeft.setPower(-1.0);
                        liftRight.setPower(-1.0);
                    }

                    liftState = 14;
                    liftPosition = 1;

                } else if (gamepad2.dpad_down && (liftLeft.getTargetPosition() != (-alliance_middle))) {

                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftLeft.setTargetPosition(-alliance_middle);
                    liftRight.setTargetPosition(-alliance_middle);
                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLeft.setPower(-1.0);
                    liftRight.setPower(-1.0);

                    if (liftLeft.getCurrentPosition() < (-alliance_middle)) {

                        liftLeft.setPower(1.0);
                        liftRight.setPower(1.0);

                    } else {

                        liftLeft.setPower(-1.0);
                        liftRight.setPower(-1.0);
                    }

                    liftState = 15;
                    liftPosition = 1;

                } else if (gamepad2.dpad_left && (liftLeft.getTargetPosition() != (-alliance_targetBalanced))) {

                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftLeft.setTargetPosition(-alliance_targetBalanced);
                    liftRight.setTargetPosition(-alliance_targetBalanced);
                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLeft.setPower(-1.0);
                    liftRight.setPower(-1.0);

                    if (liftLeft.getCurrentPosition() < (-alliance_targetBalanced)) {

                        liftLeft.setPower(1.0);
                        liftRight.setPower(1.0);

                    } else {

                        liftLeft.setPower(-1.0);
                        liftRight.setPower(-1.0);
                    }

                    liftState = 16;
                    liftPosition = 1;

                } else if (gamepad2.dpad_up && (liftLeft.getTargetPosition() != (-alliance_targetTipped))) {

                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftLeft.setTargetPosition(-alliance_targetTipped);
                    liftRight.setTargetPosition(-alliance_targetTipped);
                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLeft.setPower(-1.0);
                    liftRight.setPower(-1.0);

                    if (liftLeft.getCurrentPosition() < (-alliance_targetTipped)) {

                        liftLeft.setPower(1.0);
                        liftRight.setPower(1.0);

                    } else {

                        liftLeft.setPower(-1.0);
                        liftRight.setPower(-1.0);
                    }

                    liftState = 17;
                    liftPosition = 1;
                }
            } else if (liftState == 21) {
                if (liftTime.milliseconds() > 700/* || (d_bendLeft.getPosition() == d_minRange_bendLeft)*/) {

                    bucket.setPosition(bucket_down);
                    arm.setPosition(arm_forward);

                    objectCaptured = false;

                    liftTime.reset();
                    liftState = 22;
                }

            } else if (liftState == 22) {

                if (liftTime.milliseconds() > 500 /* || liftPosition > 15 */) { // this do be risky idk
                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftLeft.setTargetPosition(0);
                    liftRight.setTargetPosition(0);
                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLeft.setPower(1.0);
                    liftRight.setPower(1.0);
                    liftState = 23;
                    // liftState = 0;
                }

            } else if (liftState == 23) {
                if (Math.abs(liftLeft.getCurrentPosition() - 0) < liftRetractError) {
                    liftPosition = 0;
                    liftState = 0;
                }
            }

            // drivetrain

            // intuitive controls in respect to the blue side of the field
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


            /** Carousel **/

            float a = gamepad2.left_trigger;
            float b = gamepad2.right_trigger;

            if (gamepad2.left_trigger != 0) {

                c_Left.setPower(0.5 + 0.5 * (int) a);
                c_Right.setPower(0.5 + 0.5 * (int) a);

            } else if (gamepad2.right_trigger != 0) {

                c_Left.setPower(-(0.5 + 0.5 * (int) a));
                c_Right.setPower(-(0.5 + 0.5 * (int) a));
            }

//            if (gamepad2.left_bumper) {
//
//                carousel.setPower(-1);
//
//            } else if (gamepad2.right_bumper) {
//
//                carousel.setPower(1);
//
//            } else {
//
//                carousel.setPower(0);
//            }

            // Manual Lift

            float c = gamepad2.left_stick_y;

            while (gamepad2.left_stick_y != 0) {

                if (c > 0) {
                    liftLeft.setTargetPosition(-590 * (int) c);
                    liftRight.setTargetPosition(-590 * (int) c);
                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLeft.setPower(-1.0);
                    liftRight.setPower(-1.0);
                }

                if (c < 0) {
                    liftLeft.setTargetPosition(0);
                    liftRight.setTargetPosition(0);
                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftLeft.setPower(1.0);
                    liftRight.setPower(1.0);
                }
            }

            // carousel.setPower(Math.max(0, Math.min(1, b-a)));

            /** Reset Encoders **/

            if (gamepad2.right_trigger != 0) {

                liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // set motor ticks to 0
                liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "leftFront (%.2f), leftBack (%.2f), rightFront (%.2f), rightBack (%.2f)",
                    leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
            telemetry.addData("Color - Left", colorSensor_left.getDistance(DistanceUnit.CM));
            telemetry.addData("Color - Right", colorSensor_right.getDistance(DistanceUnit.CM));
            telemetry.addData("Lift - Front:", liftLeft.getCurrentPosition());
            telemetry.addData("Lift - Back:", liftRight.getCurrentPosition());
            telemetry.addData("C_Left: ", c_Left.getPower());
            telemetry.addData("C_Right: ", c_Right.getPower());
            telemetry.addData("Lift State: ", liftState);
            telemetry.addData("Lift Position: ", liftPosition);
            telemetry.update();
        }
    }
}