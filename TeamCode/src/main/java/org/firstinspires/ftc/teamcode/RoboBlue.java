package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
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

@Config
@TeleOp
public class RoboBlue extends LinearOpMode {

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
    Servo turret;

    // Color sensors
    ColorRangeSensor colorSensor_left;
    ColorRangeSensor colorSensor_right;
    ColorRangeSensor bucketSensor;

    // Timers
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime leftIntakeTimer = new ElapsedTime();
    ElapsedTime rightIntakeTimer = new ElapsedTime();
    ElapsedTime carouselTimer = new ElapsedTime();
    ElapsedTime depositTimer = new ElapsedTime();

    enum IntakeState {

        IS_STATIONARY,
        IS_SETUP,
        IS_INTAKE,
        IS_CAPTURE,
        IS_TRANSFER,
        IS_OUT,
        IS_COMPLETE,
        IS_RESET_L,
        IS_RESET_R,
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

    enum CarouselState {

        CS_STATIONARY,
        CS_TURNING_LEFT,
        CS_TURNING_RIGHT
    }

    enum CarouselHand {

        CH_LEFT,
        CH_RIGHT,
        CH_NEITHER
    }

    enum LiftState {
        LS_EXTENDING,
        LS_EXTENDING_BALANCED,
        LS_EXTENDING_TIPPED,
        LS_EXTENDING_MIDDLE,
        LS_EXTENDING_NEAR,
        LS_EXTENDING_CENTER,
        LS_EXTENDING_FAR,
        LS_TARGET,
        LS_TARGET_BALANCED,
        LS_TARGET_TIPPED,
        LS_TARGET_MIDDLE,
        LS_TARGET_NEAR,
        LS_TARGET_CENTER,
        LS_TARGET_FAR,
        LS_DEPOSITING,
        LS_RELEASED,
        LS_RETRACTING
    }

    enum LiftHand {

        LH_IDLE,
        LH_BALANCED,
        LH_TIPPED,
        LH_MIDDLE,
        LH_NEAR,
        LH_CENTER,
        LH_FAR
    }

    IntakeState intakeState = IntakeState.IS_STATIONARY;
    IntakeHand intakeHand = IntakeHand.IH_NEITHER;
    BucketHand bucketHand = BucketHand.BH_CENTER;
    BumperState bumperState = BumperState.BS_OFF;
    CarouselState carouselState = CarouselState.CS_STATIONARY;
    CarouselHand carouselHand = CarouselHand.CH_NEITHER;
    LiftState liftState = LiftState.LS_EXTENDING;
    LiftHand liftHand = LiftHand.LH_IDLE;

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
        arm = hardwareMap.servo.get("arm");
        bucket = hardwareMap.servo.get("bucket");
        turret = hardwareMap.servo.get("turret");
        c_Left = hardwareMap.crservo.get("c_Left");
        c_Right = hardwareMap.crservo.get("c_Right");

        // Color sensors
        colorSensor_left = hardwareMap.get(ColorRangeSensor.class, "colorSensor_left");
        colorSensor_right = hardwareMap.get(ColorRangeSensor.class, "colorSensor_right");
        bucketSensor = hardwareMap.get(ColorRangeSensor.class, "bucketSensor");

        // Intake
        double highSweepPower = 0.8;

        // Deposit servo positions
        double bucket_down = 0.55;
        double bucket_left = 0.27;
        double bucket_right = 0.83;
        double arm_forward = 0.30;   // temporary
        double arm_backward = 0.87;
        double turret_center = 0.5;

        double i_minRange_topLeft = 0.15;
        double i_maxRange_topLeft = 0.92;
        double i_minRange_bottomLeft = 0.90;
        double i_maxRange_bottomLeft = 0.13;
        double i_minRange_topRight = 0.87;
        double i_maxRange_topRight = 0.16;
        double i_minRange_bottomRight = 0.16;
        double i_maxRange_bottomRight = 0.87;

        // Carousel
        double maxSpinPower = 0.5;

        // Factor
        double normalSpeed = 1.0;
        int TARGET_TIPPED = 625;
        int TARGET_BALANCED = 350;
        int TARGET_MIDDLE = 550;
        int TARGET_NEAR = 120;
        int TARGET_CENTER = 200;
        int TARGET_FAR = 280;

        // Reset encoders
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // set motor ticks to 0
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

                        intakeHand = IntakeHand.IH_LEFT;
                        intakeState = IntakeState.IS_SETUP;

                    } else if (gamepad1.right_bumper) {

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

                        arm.setPosition(arm_backward);
                        turret.setPosition(turret_center);

                        intakeState = IntakeState.IS_INTAKE;

                    } else if (intakeHand == IntakeHand.IH_RIGHT) {

                        // right intake flips down
                        i_topRight.setPosition(i_minRange_topRight);
                        i_bottomRight.setPosition(i_minRange_bottomRight);

                        i_topLeft.setPosition(i_maxRange_topLeft);
                        i_bottomLeft.setPosition(i_maxRange_bottomLeft);

                        leftIntake.setPower(0);

                        // start right intake
                        rightIntake.setPower(-highSweepPower);

                        // turn bucket left
                        bucket.setPosition(bucket_right);

                        bucketHand = BucketHand.BH_RIGHT;

                        arm.setPosition(arm_backward);
                        turret.setPosition(turret_center);

                        intakeState = IntakeState.IS_INTAKE;
                    }

                    break;

                case IS_INTAKE:

                    if (gamepad1.left_trigger != 0) {

                        intakeState = IntakeState.IS_RESET_L;

                    } else if (gamepad1.right_trigger != 0) {

                        intakeState = IntakeState.IS_RESET_R;

                    } else if (intakeHand == IntakeHand.IH_LEFT) {

                        if (colorSensor_left.alpha() > 500) {

                            // hold elements in intake
                            leftIntake.setPower(0.25);

                            // left intake flips up
                            i_topLeft.setPosition(i_maxRange_topLeft);
                            i_bottomLeft.setPosition(i_maxRange_bottomLeft);

                            intakeState = IntakeState.IS_CAPTURE;

                        }

                    } else if (intakeHand == IntakeHand.IH_RIGHT) {

                        if (colorSensor_right.alpha() > 500) {

                            // hold elements in intake
                            rightIntake.setPower(-0.25);

                            // left intake flips up
                            i_topRight.setPosition(i_maxRange_topRight);
                            i_bottomRight.setPosition(i_maxRange_bottomRight);

                            intakeState = IntakeState.IS_CAPTURE;

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
                            leftIntakeTimer.reset();

                        }

                    } else if (intakeHand == IntakeHand.IH_RIGHT) {

                        if (i_topRight.getPosition() == i_maxRange_topRight) {

                            intakeState = IntakeState.IS_OUT;
                            rightIntakeTimer.reset();

                        }
                    }

                    break;

                case IS_OUT:

                    if (intakeHand == IntakeHand.IH_LEFT) {

                        if (leftIntakeTimer.milliseconds() > 600) {

                            leftIntake.setPower(-highSweepPower);

                            if (bucketSensor.alpha() > 200) {

                                leftIntake.setPower(0);
                                intakeState = IntakeState.IS_COMPLETE;

                            }
                        }

                    } else if (intakeHand == IntakeHand.IH_RIGHT) {

                        if (rightIntakeTimer.milliseconds() > 800) {

                            rightIntake.setPower(highSweepPower);

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

                case IS_RESET_L:

                    if (intakeHand == IntakeHand.IH_LEFT) {

                        i_topLeft.setPosition(i_maxRange_topLeft);
                        i_bottomLeft.setPosition(i_maxRange_bottomLeft);

                        leftIntake.setPower(0);

                        // turn bucket left
                        bucket.setPosition(bucket_down);

                        bucketHand = BucketHand.BH_CENTER;

                        intakeState = IntakeState.IS_STATIONARY;
                    }

                    break;

                case IS_RESET_R:

                    if (intakeHand == IntakeHand.IH_RIGHT) {

                        i_topRight.setPosition(i_maxRange_topRight);
                        i_bottomRight.setPosition(i_maxRange_bottomRight);

                        rightIntake.setPower(0);

                        // turn bucket left
                        bucket.setPosition(bucket_down);

                        bucketHand = BucketHand.BH_CENTER;

                        intakeState = IntakeState.IS_STATIONARY;
                    }

                    break;
            }


//            /** Lift **/
//
//            // Motor tick count is equal to 384.5
//
//            switch (liftState) {
//                case LS_EXTENDING:
//
//                    // Check if button was pressed
//                    if (gamepad2.dpad_up) {
//
//                        liftHand = LiftHand.LH_TIPPED;
//
//                        // Run lift
//                        liftLeft.setTargetPosition(-TARGET_TIPPED);
//                        liftRight.setTargetPosition(-TARGET_TIPPED);
//                        liftLeft.setPower(-1.0);
//                        liftRight.setPower(-1.0);
//
//                        liftState = LiftState.LS_TARGET;
//
//                    } else if (gamepad2.dpad_left) {
//
//                        liftHand = liftHand.LH_BALANCED;
//
//                        // Run lift
//                        liftLeft.setTargetPosition(-TARGET_BALANCED);
//                        liftRight.setTargetPosition(-TARGET_BALANCED);
//                        liftLeft.setPower(-1.0);
//                        liftRight.setPower(-1.0);
//
//                        liftState = LiftState.LS_TARGET;
//
//                    } else if (gamepad2.y) {
//
//                        liftHand = LiftHand.LH_FAR;
//
//                        // Run lift
//                        liftLeft.setTargetPosition(-TARGET_FAR);
//                        liftRight.setTargetPosition(-TARGET_FAR);
//                        liftLeft.setPower(-1.0);
//                        liftRight.setPower(-1.0);
//
//                        liftState = LiftState.LS_TARGET;
//
//                    } else if (gamepad2.b) {
//
//                        liftHand = LiftHand.LH_CENTER;
//
//                        // Run lift
//                        liftLeft.setTargetPosition(-TARGET_CENTER);
//                        liftRight.setTargetPosition(-TARGET_CENTER);
//                        liftLeft.setPower(-1.0);
//                        liftRight.setPower(-1.0);
//
//                        liftState = LiftState.LS_TARGET;
//
//                    } else if (gamepad2.a) {
//
//                        liftHand = LiftHand.LH_NEAR;
//
//                        // Run lift
//                        liftLeft.setTargetPosition(-TARGET_NEAR);
//                        liftRight.setTargetPosition(-TARGET_NEAR);
//                        liftLeft.setPower(-1.0);
//                        liftRight.setPower(-1.0);
//
//                        // Set lift state to target
//                        liftState = LiftState.LS_TARGET;
//                    }
//
//                    break;
//
//                case LS_TARGET:
//
//                    // Check if the lift has fully extended
//                    if (liftHand == LiftHand.LH_TIPPED && (Math.abs(liftLeft.getCurrentPosition() - TARGET_TIPPED)) < 10) {
//
//                        // Set state to depositing
//                        liftState = LiftState.LS_DEPOSITING;
//
//                    } else if (liftHand == LiftHand.LH_BALANCED && (Math.abs(liftLeft.getCurrentPosition() - TARGET_BALANCED)) < 10) {
//
//                        // Set state to depositing
//                        liftState = LiftState.LS_DEPOSITING;
//
//                    } else if (liftHand == LiftHand.LH_FAR && (Math.abs(liftLeft.getCurrentPosition() - TARGET_FAR)) < 5) {
//
//                        // Set state to depositing
//                        liftState = LiftState.LS_DEPOSITING;
//
//                    } else if (liftHand == LiftHand.LH_CENTER && (Math.abs(liftLeft.getCurrentPosition() - TARGET_MIDDLE)) < 5) {
//
//                        // Set state to depositing
//                        liftState = LiftState.LS_DEPOSITING;
//
//                    } else if (liftHand == LiftHand.LH_NEAR && (Math.abs(liftLeft.getCurrentPosition() - TARGET_NEAR)) < 5) {
//
//                        // Set state to depositing
//                        liftState = LiftState.LS_DEPOSITING;
//                    }
//
//                case LS_DEPOSITING:
//                    if (gamepad2.dpad_right || gamepad2.x) {
//
//                        depositTimer.reset();
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

            switch (carouselState) {

                case CS_STATIONARY:

                    if (gamepad2.left_bumper) {

                        carouselTimer.reset();

                        carouselHand = CarouselHand.CH_LEFT;

                        carouselState = CarouselState.CS_TURNING_LEFT;

                    } else if (gamepad2.right_bumper) {

                        carouselTimer.reset();

                        carouselHand = CarouselHand.CH_RIGHT;

                        carouselState = CarouselState.CS_TURNING_RIGHT;
                    }

                    break;

                case CS_TURNING_LEFT:

                    if (carouselTimer.milliseconds() < 3000) {

                        c_Left.setPower(0.3);
                        c_Right.setPower(0.3);

                    } else if (carouselTimer.milliseconds() < 4000 && carouselTimer.milliseconds() > 3000) {

                        c_Left.setPower(0.5);
                        c_Right.setPower(0.5);

                    } else if (carouselTimer.milliseconds() < 6000 && carouselTimer.milliseconds() > 4000) {

                        c_Left.setPower(1.0);
                        c_Right.setPower(1.0);

                    } else {

                        c_Left.setPower(0);
                        c_Right.setPower(0);

                        carouselState = CarouselState.CS_STATIONARY;
                    }

                case CS_TURNING_RIGHT:

                    if (carouselTimer.milliseconds() < 3000) {

                        c_Left.setPower(-0.3);
                        c_Right.setPower(-0.3);

                    } else if (carouselTimer.milliseconds() < 4000 && carouselTimer.milliseconds() > 3000) {

                        c_Left.setPower(-0.5);
                        c_Right.setPower(-0.5);

                    } else if (carouselTimer.milliseconds() < 6000 && carouselTimer.milliseconds() > 4000) {

                        c_Left.setPower(-1.0);
                        c_Right.setPower(-1.0);

                    } else {

                        c_Left.setPower(0);
                        c_Right.setPower(0);
                    }

                    break;
            }

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

            // Manual Lift

            float c = gamepad2.right_stick_y;

            if (c > 0) {

                liftLeft.setTargetPosition(400);
                liftRight.setTargetPosition(400);
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
//            telemetry.addData("Lift Position: ", liftPosition);
            telemetry.addData("C: ", c);
            telemetry.update();
        }
    }
}