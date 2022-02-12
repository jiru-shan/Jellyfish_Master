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
public class RoboBlueTest extends LinearOpMode {

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
    ColorRangeSensor bucketSensor;

    // Runtime
    ElapsedTime runtime = new ElapsedTime();

    enum IntakeState {

        IS_STATIONARY,
        IS_SETUP,
        IS_INTAKE,
        IS_CAPTURE,
        IS_TRANSFER,
        IS_OUT,
        IS_COMPLETE,
        IS_LIFT
    }

    enum IntakeHand {

        IH_LEFT,
        IH_RIGHT,
        IH_NEITHER
    }

    enum BucketHand {

        BH_LEFT,
        BH_RIGHT,
        BH_CENTER
    }

    enum BumperState {

        BS_OFF,
        BS_ON
    }

    //    enum LiftState {
//        LIFT_EXTENDING,
//        LIFT_EXTENDING_BALANCED,
//        LIFT_EXTENDING_TIPPED,
//        LIFT_EXTENDING_NEAR,
//        LIFT_EXTENDING_MIDDLE,
//        LIFT_EXTENDING_FAR,
//        LIFT_TARGET,
//        LIFT_TARGET_BALANCED,
//        LIFT_TARGET_TIPPED,
//        LIFT_TARGET_NEAR,
//        LIFT_TARGET_MIDDLE,
//        LIFT_TARGET_FAR,
//        LIFT_DEPOSITING,
//        LIFT_RELEASED,
//        LIFT_RETRACTING
//    }

    IntakeState intakeState = IntakeState.IS_STATIONARY;
    IntakeHand intakeHand = IntakeHand.IH_NEITHER;
    BucketHand bucketHand = BucketHand.BH_CENTER;
    BumperState bumperState = BumperState.BS_OFF;

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
        bucketSensor = hardwareMap.get(ColorRangeSensor.class, "bucketSensor");

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
        double i_maxRange_topLeft = 0.92;
        double i_minRange_bottomLeft = 0.90;   // 0.92
        double i_maxRange_bottomLeft = 0.13;
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

        ElapsedTime leftIntakeTime = new ElapsedTime();
        ElapsedTime rightIntakeTime = new ElapsedTime();

        double intakeCaptureDistance = 6.0;
        double intakeEjectDistance = 30.0;
        int intakeFlipUpTime = 250;

        int liftState = 0;
        int liftPosition = 0;
        int liftExtendError = 100;
        int liftRetractError = 10;
        ElapsedTime liftTime = new ElapsedTime();

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

//            i_topLeft.setPosition(i_maxRange_topLeft);
//            i_bottomLeft.setPosition(i_maxRange_bottomLeft);
//            i_topRight.setPosition(i_maxRange_topRight);
//            i_bottomRight.setPosition(i_maxRange_bottomRight);

//            bucket.setPosition(bucket_down);

            /** Combined Functions **/

            // Resting position - up
            // With a button, intake should flip down, start intake, check if object present, lift up, ex-take into deposit
            // During this time, one flap of deposit should be up and one should be down

            switch (intakeState) {

                case IS_STATIONARY:

                    if (gamepad1.left_bumper) {

                        bumperState = BumperState.BS_ON;
                        intakeHand = IntakeHand.IH_LEFT;
                        intakeState = IntakeState.IS_SETUP;

                    } else if (gamepad1.right_bumper) {

                        bumperState = BumperState.BS_ON;
                        intakeHand = IntakeHand.IH_RIGHT;
                        intakeState = IntakeState.IS_SETUP;

                    }

                    break;

                case IS_SETUP:

                    if (intakeHand == IntakeHand.IH_LEFT) {

                        // left intake flips down
                        i_topLeft.setPosition(i_minRange_topLeft);
                        i_bottomLeft.setPosition(i_minRange_bottomLeft);

                        i_topRight.setPosition(i_maxRange_topRight);
                        i_bottomRight.setPosition(i_maxRange_bottomRight);

                        rightIntake.setPower(0);

                        leftIntake.setPower(highSweepPower);

                        // turn bucket left
                        bucket.setPosition(bucket_left);

                        bucketHand = BucketHand.BH_LEFT;

                        intakeState = IntakeState.IS_INTAKE;

                    } else if (intakeHand == IntakeHand.IH_RIGHT) {

                        // right intake flips down
                        i_topRight.setPosition(i_minRange_topRight);
                        i_bottomRight.setPosition(i_minRange_bottomRight);

                        i_topLeft.setPosition(i_maxRange_topLeft);
                        i_bottomLeft.setPosition(i_maxRange_bottomLeft);

                        leftIntake.setPower(0);

                        // start right intake
                        rightIntake.setPower(highSweepPower);

                        // turn bucket left
                        bucket.setPosition(bucket_right);

                        bucketHand = BucketHand.BH_RIGHT;

                        intakeState = IntakeState.IS_INTAKE;
                    }

                    break;

                case IS_INTAKE:

                    if (intakeHand == IntakeHand.IH_LEFT) {

                        if (colorSensor_left.alpha() > 500) {

                            // hold elements in intake
                            leftIntake.setPower(0.25);

                            // left intake flips up
                            i_topLeft.setPosition(i_maxRange_topLeft);
                            i_bottomLeft.setPosition(i_maxRange_bottomLeft);

                            intakeState = IntakeState.IS_CAPTURE;

                        } else if (gamepad1.left_bumper) {

                            bumperState = BumperState.BS_OFF;

                            i_topLeft.setPosition(i_maxRange_topLeft);
                            i_bottomLeft.setPosition(i_maxRange_bottomLeft);

                            leftIntake.setPower(0);

                            // turn bucket left
                            bucket.setPosition(bucket_down);

                            bucketHand = BucketHand.BH_CENTER;

                            intakeState = IntakeState.IS_STATIONARY;

                        }

                    } else if (intakeHand == IntakeHand.IH_RIGHT) {

                        if (colorSensor_right.alpha() > 500) {

                            // hold elements in intake
                            rightIntake.setPower(0.25);

                            // left intake flips up
                            i_topRight.setPosition(i_maxRange_topRight);
                            i_bottomRight.setPosition(i_maxRange_bottomRight);

                            intakeState = IntakeState.IS_CAPTURE;

                        } else if (gamepad1.right_bumper) {

                            bumperState = BumperState.BS_OFF;

                            i_topRight.setPosition(i_maxRange_topRight);
                            i_bottomRight.setPosition(i_maxRange_bottomRight);

                            rightIntake.setPower(0);

                            // turn bucket left
                            bucket.setPosition(bucket_down);

                            bucketHand = BucketHand.BH_CENTER;

                            intakeState = IntakeState.IS_STATIONARY;
                        }
                    }

                    break;

                case IS_CAPTURE:

                    if (intakeHand == IntakeHand.IH_LEFT) {

                        if (i_topLeft.getPosition() == i_maxRange_topLeft) {

                            // stop left intake
                            leftIntake.setPower(0);

                            intakeState = IntakeState.IS_TRANSFER;

                        }

                    } else if (intakeHand == IntakeHand.IH_RIGHT) {

                        if (i_topRight.getPosition() == i_maxRange_topRight) {

                            // stop left intake
                            rightIntake.setPower(0);

                            intakeState = IntakeState.IS_TRANSFER;

                        }
                    }

                    break;


                case IS_TRANSFER:

                    if (intakeHand == IntakeHand.IH_LEFT) {

                        if (i_topLeft.getPosition() == i_maxRange_topLeft) {

                            intakeState = IntakeState.IS_OUT;
                            leftIntakeTime.reset();

                        }

                    } else if (intakeHand == IntakeHand.IH_RIGHT) {

                        if (i_topRight.getPosition() == i_maxRange_topRight) {

                            intakeState = IntakeState.IS_OUT;
                            rightIntakeTime.reset();

                        }
                    }

                    break;

                case IS_OUT:

                    if (intakeHand == IntakeHand.IH_LEFT) {

                        if (leftIntakeTime.milliseconds() > 500) {

                            leftIntake.setPower(-highSweepPower);

                            if (bucketSensor.alpha() > 200) {

                                leftIntake.setPower(0);
                                intakeState = IntakeState.IS_COMPLETE;

                            }
                        }

                    } else if (intakeHand == IntakeHand.IH_RIGHT) {

                        if (rightIntakeTime.milliseconds() > 800) {

                            rightIntake.setPower(-highSweepPower);

                            if (bucketSensor.alpha() > 200) {

                                rightIntake.setPower(0);
                                intakeState = IntakeState.IS_COMPLETE;

                            }
                        }
                    }

                    break;

                case IS_COMPLETE:

                    if (intakeHand == IntakeHand.IH_LEFT) {

                        bucket.setPosition(bucket_down);
                        bucketHand = BucketHand.BH_CENTER;
                        intakeState = IntakeState.IS_STATIONARY;
                        intakeHand = IntakeHand.IH_NEITHER;

                    } else if (intakeHand == IntakeHand.IH_RIGHT) {

                            bucket.setPosition(bucket_down);
                            bucketHand = BucketHand.BH_CENTER;
                            intakeState = IntakeState.IS_STATIONARY;
                            intakeHand = IntakeHand.IH_NEITHER;
                    }

                    break;
            }


            /** Lift **/
//
//            // Motor tick count is equal to 384.5
//
//            switch (liftState) {
//                case LIFT_EXTENDING:
//
//                    // Check if button was pressed
//                    if (gamepad2.dpad_up) {
//
//                        // Run lift
//                        liftFront.setTargetPosition(-TARGET_TIPPED);
//                        liftBack.setTargetPosition(-TARGET_TIPPED);
//                        liftFront.setPower(-1.0);
//                        liftBack.setPower(-1.0);
//
//                        // "10" is arbitrary - might need to be adjusted
//                        if ((Math.abs(liftFront.getCurrentPosition() - TARGET_TIPPED)) > 10) {
//
//                            // Run lift
//                            liftFront.setTargetPosition(-TARGET_TIPPED);
//                            liftBack.setTargetPosition(-TARGET_TIPPED);
//                            liftFront.setPower(-1.0);
//                            liftBack.setPower(-1.0);
//
//                            // Ensure movement of drivetrain during while loop
//                            y = -gamepad1.right_stick_x; // Reversed
//                            x = gamepad1.left_stick_x * 1.1; // Strafing + Precision
//                            rx = -gamepad1.left_stick_y; // Forward/Backward
//
//                            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//                            leftFrontPower = (y + x - rx) / denominator;
//                            leftBackPower = (y - x - rx) / denominator;
//                            rightFrontPower = (y - x + rx) / denominator;
//                            rightBackPower = (y + x + rx) / denominator;
//
//                            leftFront.setPower(leftBackPower);
//                            leftBack.setPower(leftFrontPower);
//                            rightFront.setPower(rightFrontPower);
//                            rightBack.setPower(rightBackPower);
//
//                            // Set lift state to target
//                            liftState = LiftState.LIFT_TARGET;
//
//                        } else {
//
//                            liftState = LiftState.LIFT_EXTENDING;
//                        }
//
//                        break;
//
//                    } else if (gamepad2.dpad_left) {
//
//                        // Run lift
//                        liftFront.setTargetPosition(-TARGET_TIPPED);
//                        liftBack.setTargetPosition(-TARGET_TIPPED);
//                        liftFront.setPower(-1.0);
//                        liftBack.setPower(-1.0);
//
//                        // "10" is arbitrary - might need to be adjusted
//                        if ((Math.abs(liftFront.getCurrentPosition() - TARGET_BALANCED)) > 10) {
//
//                            // Run lift
//                            liftFront.setPower(-1.0);
//                            liftBack.setPower(-1.0);
//                            liftFront.setTargetPosition(-TARGET_BALANCED);
//                            liftBack.setTargetPosition(-TARGET_BALANCED);
//
//                            // Ensure movement of drivetrain during while loop
//                            y = -gamepad1.right_stick_x; // Reversed
//                            x = gamepad1.left_stick_x * 1.1; // Strafing + Precision
//                            rx = -gamepad1.left_stick_y; // Forward/Backward
//
//                            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//                            leftFrontPower = (y + x - rx) / denominator;
//                            leftBackPower = (y - x - rx) / denominator;
//                            rightFrontPower = (y - x + rx) / denominator;
//                            rightBackPower = (y + x + rx) / denominator;
//
//                            leftFront.setPower(leftBackPower);
//                            leftBack.setPower(leftFrontPower);
//                            rightFront.setPower(rightFrontPower);
//                            rightBack.setPower(rightBackPower);
//
//                            // Set lift state to target
//                            liftState = LiftState.LIFT_TARGET;
//
//                        } else {
//
//                            liftState = LiftState.LIFT_EXTENDING;
//                        }
//
//                        break;
//
//                    } else if (gamepad2.y) {
//
//                        // Run lift
//                        liftFront.setTargetPosition(-TARGET_TIPPED);
//                        liftBack.setTargetPosition(-TARGET_TIPPED);
//                        liftFront.setPower(-1.0);
//                        liftBack.setPower(-1.0);
//
//                        // "10" is arbitrary - might need to be adjusted
//                        if ((Math.abs(liftFront.getCurrentPosition() - TARGET_FAR) > 5)) {
//
//                            // Run lift
//                            liftFront.setTargetPosition(-TARGET_FAR);
//                            liftBack.setTargetPosition(-TARGET_FAR);
//                            liftFront.setPower(-1.0);
//                            liftBack.setPower(-1.0);
//
//                            // Ensure movement of drivetrain during while loop
//                            y = -gamepad1.right_stick_x; // Reversed
//                            x = gamepad1.left_stick_x * 1.1; // Strafing + Precision
//                            rx = -gamepad1.left_stick_y; // Forward/Backward
//
//                            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//                            leftFrontPower = (y + x - rx) / denominator;
//                            leftBackPower = (y - x - rx) / denominator;
//                            rightFrontPower = (y - x + rx) / denominator;
//                            rightBackPower = (y + x + rx) / denominator;
//
//                            leftFront.setPower(leftBackPower);
//                            leftBack.setPower(leftFrontPower);
//                            rightFront.setPower(rightFrontPower);
//                            rightBack.setPower(rightBackPower);
//
//                            // Set lift state to target
//                            liftState = LiftState.LIFT_TARGET;
//
//                        } else {
//
//                            liftState = LiftState.LIFT_EXTENDING;
//                        }
//
//                        break;
//
//                    } else if (gamepad2.b) {
//
//                        // Run lift
//                        liftFront.setTargetPosition(-TARGET_TIPPED);
//                        liftBack.setTargetPosition(-TARGET_TIPPED);
//                        liftFront.setPower(-1.0);
//                        liftBack.setPower(-1.0);
//
//                        // "10" is arbitrary - might need to be adjusted
//                        if ((Math.abs(liftFront.getCurrentPosition() - TARGET_MIDDLE)) > 5) {
//
//                            // Run lift
//                            liftFront.setTargetPosition(-TARGET_MIDDLE);
//                            liftBack.setTargetPosition(-TARGET_MIDDLE);
//                            liftFront.setPower(-1.0);
//                            liftBack.setPower(-1.0);
//
//                            // Ensure movement of drivetrain during while loop
//                            y = -gamepad1.right_stick_x; // Reversed
//                            x = gamepad1.left_stick_x * 1.1; // Strafing + Precision
//                            rx = -gamepad1.left_stick_y; // Forward/Backward
//
//                            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//                            leftFrontPower = (y + x - rx) / denominator;
//                            leftBackPower = (y - x - rx) / denominator;
//                            rightFrontPower = (y - x + rx) / denominator;
//                            rightBackPower = (y + x + rx) / denominator;
//
//                            leftFront.setPower(leftBackPower);
//                            leftBack.setPower(leftFrontPower);
//                            rightFront.setPower(rightFrontPower);
//                            rightBack.setPower(rightBackPower);
//
//                            // Set lift state to target
//                            liftState = LiftState.LIFT_TARGET;
//
//                        } else {
//
//                            liftState = LiftState.LIFT_EXTENDING;
//
//                        }
//
//                        break;
//
//                    } else if (gamepad2.a) {
//
//                        // Run lift
//                        liftFront.setTargetPosition(-TARGET_TIPPED);
//                        liftBack.setTargetPosition(-TARGET_TIPPED);
//                        liftFront.setPower(-1.0);
//                        liftBack.setPower(-1.0);
//
//                        // "10" is arbitrary - might need to be adjusted
//                        if ((Math.abs(liftFront.getCurrentPosition() - TARGET_NEAR) > 5)) {
//
//                            // Run lift
//                            liftFront.setTargetPosition(-TARGET_NEAR);
//                            liftBack.setTargetPosition(-TARGET_NEAR);
//                            liftFront.setPower(-1.0);
//                            liftBack.setPower(-1.0);
//
//                            // Ensure movement of drivetrain during while loop
//                            y = -gamepad1.right_stick_x; // Reversed
//                            x = gamepad1.left_stick_x * 1.1; // Strafing + Precision
//                            rx = -gamepad1.left_stick_y; // Forward/Backward
//
//                            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//                            leftFrontPower = (y + x - rx) / denominator;
//                            leftBackPower = (y - x - rx) / denominator;
//                            rightFrontPower = (y - x + rx) / denominator;
//                            rightBackPower = (y + x + rx) / denominator;
//
//                            leftFront.setPower(leftBackPower);
//                            leftBack.setPower(leftFrontPower);
//                            rightFront.setPower(rightFrontPower);
//                            rightBack.setPower(rightBackPower);
//
//                            // Set lift state to target
//                            liftState = LiftState.LIFT_TARGET;
//
//                        } else {
//
//                            liftState = LiftState.LIFT_EXTENDING;
//                        }
//
//                        break;
//                    }
//
//                case LIFT_TARGET:
//
//                    // Check if the lift has fully extended
//                    if (gamepad2.dpad_up && (Math.abs(liftFront.getCurrentPosition() - TARGET_TIPPED)) < 10) {
//
//                        // Set state to depositing
//                        liftState = LiftState.LIFT_DEPOSITING;
//                        break;
//
//                    } else if (gamepad2.dpad_left && (Math.abs(liftFront.getCurrentPosition() - TARGET_BALANCED)) < 10) {
//
//                        // Set state to depositing
//                        liftState = LiftState.LIFT_DEPOSITING;
//                        break;
//
//                    } else if (gamepad2.y && (Math.abs(liftFront.getCurrentPosition() - TARGET_FAR)) < 5) {
//
//                        // Set state to depositing
//                        liftState = LiftState.LIFT_DEPOSITING;
//                        break;
//
//                    } else if (gamepad2.b && (Math.abs(liftFront.getCurrentPosition() - TARGET_MIDDLE)) < 5) {
//
//                        // Set state to depositing
//                        liftState = LiftState.LIFT_DEPOSITING;
//                        break;
//
//                    } else if (gamepad2.a && (Math.abs(liftFront.getCurrentPosition() - TARGET_NEAR)) < 5) {
//
//                        // Set state to depositing
//                        liftState = LiftState.LIFT_DEPOSITING;
//                        break;
//                    }
//
//                case LIFT_DEPOSITING:
//                    if (gamepad2.dpad_right || gamepad2.x) {
//
//                        d_open.setPosition(d_open_top);
//
//                        if (depositTimer.milliseconds() <= 500) {
//
//                            d_open.setPosition(d_open_top);
//
//                            // Keep lift in place while depositing
//                            liftFront.setTargetPosition(0);
//                            liftBack.setTargetPosition(0);
//                            liftFront.setPower(-1.0);
//                            liftBack.setPower(-1.0);
//
//                            // Ensure movement of drivetrain during while loop
//                            y = -gamepad1.right_stick_x; // Reversed
//                            x = gamepad1.left_stick_x * 1.1; // Strafing + Precision
//                            rx = -gamepad1.left_stick_y; // Forward/Backward
//
//                            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//                            leftFrontPower = (y + x - rx) / denominator;
//                            leftBackPower = (y - x - rx) / denominator;
//                            rightFrontPower = (y - x + rx) / denominator;
//                            rightBackPower = (y + x + rx) / denominator;
//
//                            leftFront.setPower(leftBackPower);
//                            leftBack.setPower(leftFrontPower);
//                            rightFront.setPower(rightFrontPower);
//                            rightBack.setPower(rightBackPower);
//
//                        }
//
//                        d_open.setPosition(d_open_minRange);
//
//                        depositTimer.reset();
//                        // Set lift state to released
//                        liftState = LiftState.LIFT_RELEASED;
//
//                        break;
//                    }
//
//                case LIFT_RELEASED:
//                    if (d_open.getPosition() == d_open_minRange) {
//
//                        // Retract lift
//                        liftFront.setTargetPosition(0);
//                        liftFront.setTargetPosition(0);
//                        liftFront.setPower(1.0);
//                        liftFront.setPower(1.0);
//
//                        if ((Math.abs(liftFront.getCurrentPosition() - LIFT_IDLE)) != 0) {
//
//                            // Retract lift
//                            liftFront.setTargetPosition(0);
//                            liftFront.setTargetPosition(0);
//                            liftFront.setPower(1.0);
//                            liftFront.setPower(1.0);
//
//                            // Ensure movement of drivetrain during while loop
//                            y = -gamepad1.right_stick_x; // Reversed
//                            x = gamepad1.left_stick_x * 1.1; // Strafing + Precision
//                            rx = -gamepad1.left_stick_y; // Forward/Backward
//
//                            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//                            leftFrontPower = (y + x - rx) / denominator;
//                            leftBackPower = (y - x - rx) / denominator;
//                            rightFrontPower = (y - x + rx) / denominator;
//                            rightBackPower = (y + x + rx) / denominator;
//
//                            leftFront.setPower(leftBackPower);
//                            leftBack.setPower(leftFrontPower);
//                            rightFront.setPower(rightFrontPower);
//                            rightBack.setPower(rightBackPower);
//
//
//                            // Set lift state to retracting
//                            liftState = LiftState.LIFT_RETRACTING;
//
//                        } else {
//
//                            liftState = LiftState.LIFT_RELEASED;
//                        }
//
//                        break;
//                    }
//
//                case LIFT_RETRACTING:
//                    // Check if lift is fully retracted
//                    if ((Math.abs(liftFront.getCurrentPosition() - LIFT_IDLE)) == 0) {
//
//                        // Set lift state to empty
//                        liftState = LiftState.LIFT_EXTENDING;
//
//                        break;
//                    }
//            }

//            if (liftState == 0) {
//
//                if (gamepad2.a) {
//
//                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftLeft.setTargetPosition(-shared_targetClose);
//                    liftRight.setTargetPosition(-shared_targetClose);
//                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    bucket.setPosition(bucket_down);
//                    liftLeft.setPower(-1.0);
//                    liftRight.setPower(-1.0);
//                    liftState = 12;
//                    liftPosition = 1;
//
//                } else if (gamepad2.b) {
//
//                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftLeft.setTargetPosition(-shared_targetMiddle);
//                    liftRight.setTargetPosition(-shared_targetMiddle);
//                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    bucket.setPosition(bucket_down);
//                    liftLeft.setPower(-1.0);
//                    liftRight.setPower(-1.0);
//                    liftState = 13;
//                    liftPosition = 1;
//
//                } else if (gamepad2.y) {
//
//                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftLeft.setTargetPosition(-shared_targetFar);
//                    liftRight.setTargetPosition(-shared_targetFar);
//                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    bucket.setPosition(bucket_down);
//                    liftLeft.setPower(-1.0);
//                    liftRight.setPower(-1.0);
//                    liftState = 14;
//                    liftPosition = 1;
//
//                } else if (gamepad2.dpad_down) {
//
//                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftLeft.setTargetPosition(-alliance_middle);
//                    liftRight.setTargetPosition(-alliance_middle);
//                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    bucket.setPosition(bucket_down);
//                    liftLeft.setPower(-1.0);
//                    liftRight.setPower(-1.0);
//                    liftState = 15;
//                    liftPosition = 1;
//
//                } else if (gamepad2.dpad_left) {
//
//                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftLeft.setTargetPosition(-alliance_targetBalanced);
//                    liftRight.setTargetPosition(-alliance_targetBalanced);
//                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    bucket.setPosition(bucket_down);
//                    liftLeft.setPower(-1.0);
//                    liftRight.setPower(-1.0);
//                    liftState = 16;
//                    liftPosition = 1;
//
//                } else if (gamepad2.dpad_up) {
//
//                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftLeft.setTargetPosition(-alliance_targetTipped);
//                    liftRight.setTargetPosition(-alliance_targetTipped);
//                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    bucket.setPosition(bucket_down);
//                    liftLeft.setPower(-1.0);
//                    liftRight.setPower(-1.0);
//                    liftState = 17;
//                    liftPosition = 1;
//                }
//
//            } else if (gamepad2.left_trigger != 0 && liftLeft.getTargetPosition() < 500) {
//
//                bucket.setPosition(bucket_down);
//                liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                liftLeft.setTargetPosition(800);
//                liftRight.setTargetPosition(800);
//                liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                liftLeft.setPower(1.0);
//                liftRight.setPower(1.0);
//                liftState = 0;
//
//            } else if (gamepad2.x && liftState < 20) {
//
//                if (Math.abs(liftLeft.getCurrentPosition()) > 100) {
//
//                    arm.setPosition(arm_forward);
//                    bucket.setPosition(bucket_left);
//
//                    liftState = 21;
//                    liftTime.reset();
//                }
//
//            } else if (gamepad2.dpad_right && liftState < 20) {
//
//                if (Math.abs(liftLeft.getCurrentPosition()) > 100) {
//
//                    arm.setPosition(arm_forward);
//                    bucket.setPosition(bucket_right);
//
//                    liftState = 21;
//                    liftTime.reset();
//                }
//
//            } else if (liftState >= 10 && liftState < 20) {
//
//                if (gamepad2.a && (liftLeft.getTargetPosition() != (-shared_targetClose))) {
//
//                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftLeft.setTargetPosition(-shared_targetClose);
//                    liftRight.setTargetPosition(-shared_targetClose);
//                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                    if (liftLeft.getCurrentPosition() < (-shared_targetClose)) {
//
//                        liftLeft.setPower(1.0);
//                        liftRight.setPower(1.0);
//
//                    } else {
//
//                        liftLeft.setPower(-1.0);
//                        liftRight.setPower(-1.0);
//                    }
//
//                    liftState = 12;
//                    liftPosition = 1;
//
//                } else if (gamepad2.b && (liftLeft.getTargetPosition() != (-shared_targetMiddle))) {
//
//                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftLeft.setTargetPosition(-shared_targetMiddle);
//                    liftRight.setTargetPosition(-shared_targetMiddle);
//                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    liftLeft.setPower(-1.0);
//                    liftRight.setPower(-1.0);
//
//                    if (liftLeft.getCurrentPosition() < (-shared_targetMiddle)) {
//
//                        liftLeft.setPower(1.0);
//                        liftRight.setPower(1.0);
//
//                    } else {
//
//                        liftLeft.setPower(-1.0);
//                        liftRight.setPower(-1.0);
//                    }
//
//                    liftState = 13;
//                    liftPosition = 1;
//
//                } else if (gamepad2.y && (liftLeft.getTargetPosition() != (-shared_targetFar))) {
//
//                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftLeft.setTargetPosition(-shared_targetFar);
//                    liftRight.setTargetPosition(-shared_targetFar);
//                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    liftLeft.setPower(-1.0);
//                    liftRight.setPower(-1.0);
//
//                    if (liftLeft.getCurrentPosition() < (-shared_targetFar)) {
//
//                        liftLeft.setPower(1.0);
//                        liftRight.setPower(1.0);
//
//                    } else {
//
//                        liftLeft.setPower(-1.0);
//                        liftRight.setPower(-1.0);
//                    }
//
//                    liftState = 14;
//                    liftPosition = 1;
//
//                } else if (gamepad2.dpad_down && (liftLeft.getTargetPosition() != (-alliance_middle))) {
//
//                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftLeft.setTargetPosition(-alliance_middle);
//                    liftRight.setTargetPosition(-alliance_middle);
//                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    liftLeft.setPower(-1.0);
//                    liftRight.setPower(-1.0);
//
//                    if (liftLeft.getCurrentPosition() < (-alliance_middle)) {
//
//                        liftLeft.setPower(1.0);
//                        liftRight.setPower(1.0);
//
//                    } else {
//
//                        liftLeft.setPower(-1.0);
//                        liftRight.setPower(-1.0);
//                    }
//
//                    liftState = 15;
//                    liftPosition = 1;
//
//                } else if (gamepad2.dpad_left && (liftLeft.getTargetPosition() != (-alliance_targetBalanced))) {
//
//                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftLeft.setTargetPosition(-alliance_targetBalanced);
//                    liftRight.setTargetPosition(-alliance_targetBalanced);
//                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    liftLeft.setPower(-1.0);
//                    liftRight.setPower(-1.0);
//
//                    if (liftLeft.getCurrentPosition() < (-alliance_targetBalanced)) {
//
//                        liftLeft.setPower(1.0);
//                        liftRight.setPower(1.0);
//
//                    } else {
//
//                        liftLeft.setPower(-1.0);
//                        liftRight.setPower(-1.0);
//                    }
//
//                    liftState = 16;
//                    liftPosition = 1;
//
//                } else if (gamepad2.dpad_up && (liftLeft.getTargetPosition() != (-alliance_targetTipped))) {
//
//                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftLeft.setTargetPosition(-alliance_targetTipped);
//                    liftRight.setTargetPosition(-alliance_targetTipped);
//                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    liftLeft.setPower(-1.0);
//                    liftRight.setPower(-1.0);
//
//                    if (liftLeft.getCurrentPosition() < (-alliance_targetTipped)) {
//
//                        liftLeft.setPower(1.0);
//                        liftRight.setPower(1.0);
//
//                    } else {
//
//                        liftLeft.setPower(-1.0);
//                        liftRight.setPower(-1.0);
//                    }
//
//                    liftState = 17;
//                    liftPosition = 1;
//                }
//            } else if (liftState == 21) {
//                if (liftTime.milliseconds() > 700/* || (d_bendLeft.getPosition() == d_minRange_bendLeft)*/) {
//
//                    bucket.setPosition(bucket_down);
//                    arm.setPosition(arm_forward);
//
//                    objectCaptured = false;
//
//                    liftTime.reset();
//                    liftState = 22;
//                }
//
//            } else if (liftState == 22) {
//
//                if (liftTime.milliseconds() > 500 /* || liftPosition > 15 */) { // this do be risky idk
//                    liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    liftLeft.setTargetPosition(0);
//                    liftRight.setTargetPosition(0);
//                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    liftLeft.setPower(1.0);
//                    liftRight.setPower(1.0);
//                    liftState = 23;
//                    // liftState = 0;
//                }
//
//            } else if (liftState == 23) {
//                if (Math.abs(liftLeft.getCurrentPosition() - 0) < liftRetractError) {
//                    liftPosition = 0;
//                    liftState = 0;
//                }
//            }

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

            if (gamepad2.left_trigger > 0.05) {

                c_Left.setPower(0.5 * a);
                c_Right.setPower(0.5 * a);

            } else if (gamepad2.right_trigger > 0.05) {

                c_Left.setPower(-(0.5 * b));
                c_Right.setPower(-(0.5 * b));

            } else {

                c_Left.setPower(0);
                c_Right.setPower(0);
            }

            if (gamepad2.left_bumper) {

                c_Left.setPower(1);
                c_Right.setPower(1);

            } else if (gamepad2.right_bumper) {

                c_Left.setPower(-1);
                c_Right.setPower(-1);

            } else {

                c_Left.setPower(0);
                c_Right.setPower(0);
            }

            // Manual Lift

            float c = gamepad2.left_stick_y;

            if (c > 0) {

                liftLeft.setTargetPosition(-590 * (int) c);
                liftRight.setTargetPosition(-590 * (int) c);
                liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftLeft.setPower(-1.0);
                liftRight.setPower(-1.0);

            } else if (c < 0) {

                liftLeft.setTargetPosition(0);
                liftRight.setTargetPosition(0);
                liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftLeft.setPower(1.0);
                liftRight.setPower(1.0);

            }

            /** Reset Encoders **/

            if (gamepad2.right_trigger != 0) {

//                liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // set motor ticks to 0
//                liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "leftFront (%.2f), leftBack (%.2f), rightFront (%.2f), rightBack (%.2f)",
                    leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
            telemetry.addData("Left Sensor: ", colorSensor_left.alpha());
            telemetry.addData("Right Sensor: ", colorSensor_right.alpha());
            telemetry.addData("BucketSensor: ", bucketSensor.alpha());
            telemetry.addData("Intake State: ", intakeState);
//            telemetry.addData("Lift - Front:", liftLeft.getCurrentPosition());
//            telemetry.addData("Lift - Back:", liftRight.getCurrentPosition());
            telemetry.addData("C_Left: ", c_Left.getPower());
            telemetry.addData("C_Right: ", c_Right.getPower());
            telemetry.addData("Lift State: ", liftState);
            telemetry.addData("Lift Position: ", liftPosition);
            telemetry.update();
        }
    }
}