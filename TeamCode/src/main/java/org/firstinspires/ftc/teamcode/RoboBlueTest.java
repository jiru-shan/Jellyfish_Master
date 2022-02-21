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
    ElapsedTime turretTimer = new ElapsedTime();

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

    enum LiftState {

        LS_STATIONARY,
        LS_EXTENDING,
        LS_EXTENDING_BALANCED,
        LS_EXTENDING_TIPPED,
        LS_EXTENDING_MIDDLE,
        LS_EXTENDING_NEAR,
        LS_EXTENDING_CENTER,
        LS_EXTENDING_FAR,
        LS_TURRET_DOWN,
        LS_TURRET_UP,
        LS_TURRET_CHECK,
        LS_TARGET,
        LS_TARGET_BALANCED,
        LS_TARGET_TIPPED,
        LS_TARGET_MIDDLE,
        LS_TARGET_NEAR,
        LS_TARGET_CENTER,
        LS_TARGET_FAR,
        LS_DEPOSITING_ALLIANCE,
        LS_DEPOSITING_SHARED,
        LS_RELEASED,
        LS_RETRACTING,
        LS_IDLE
    }

    enum ManualLiftState {

        MLS_STATIONARY,
        MLS_EXTENDING,
        MLS_RETRACTING,
        MLS_ARM_CHECK,
        MLS_IDLE
    }

    enum LiftHand {

        LH_IDLE,
        LH_BALANCED,
        LH_TIPPED,
        LH_MIDDLE,
        LH_NEAR,
        LH_CENTER,
        LH_FAR,
        LH_RETRACT_ALLIANCE,
        LH_RETRACT_SHARED
    }

    // State Declarations
    IntakeState intakeState = IntakeState.IS_STATIONARY;
    IntakeHand intakeHand = IntakeHand.IH_NEITHER;
    BucketHand bucketHand = BucketHand.BH_CENTER;
    LiftState liftState = LiftState.LS_STATIONARY;
    LiftHand liftHand = LiftHand.LH_IDLE;
    ManualLiftState manualLiftState = ManualLiftState.MLS_STATIONARY;

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
        double bucket_down = 0.48;
        double bucket_left = 0.20;
        double bucket_right = 0.76;
        double arm_forward_alliance = 0.30;
        double arm_forward_shared = 0.10;
        double arm_intermediate = 0.70;
        double arm_backward = 0.92;  // 0.92
        double turret_center = 0.49;
        double turret_target = 0.20;

        // Servo positions
        double i_minRange_topLeft = 0.15;
        double i_maxRange_topLeft = 0.92;
        double i_minRange_bottomLeft = 0.90;
        double i_maxRange_bottomLeft = 0.13;
        double i_minRange_topRight = 0.92;
        double i_maxRange_topRight = 0.16;
        double i_minRange_bottomRight = 0.12;
        double i_maxRange_bottomRight = 0.88;

        // Carousel
        double maxSpinPower = 0.5;

        // Factor
        double normalSpeed = 1.0;

        // Lift Positions
        int TARGET_TIPPED = 340;
        int TARGET_BALANCED = 320;
        int TARGET_MIDDLE = 210;
        int TARGET_NEAR = 120;
        int TARGET_CENTER = 160;
        int TARGET_FAR = 200;

        // Reset encoders
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // set motor ticks to 0
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bucket.setPosition(bucket_down);
        arm.setPosition(arm_backward);
        turret.setPosition(turret_center);

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
                        rightIntake.setPower(highSweepPower);

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

                        } else if (gamepad1.right_bumper) {

                            intakeHand = IntakeHand.IH_RIGHT;
                            intakeState = IntakeState.IS_SETUP;
                        }

                    } else if (intakeHand == IntakeHand.IH_RIGHT) {


                        if (colorSensor_right.alpha() > 500) {

                            // hold elements in intake
                            rightIntake.setPower(-0.25);

                            // left intake flips up
                            i_topRight.setPosition(i_maxRange_topRight);
                            i_bottomRight.setPosition(i_maxRange_bottomRight);

                            intakeState = IntakeState.IS_CAPTURE;

                        } else if (gamepad1.left_bumper) {

                            intakeHand = IntakeHand.IH_LEFT;
                            intakeState = IntakeState.IS_SETUP;
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

                        if (rightIntakeTimer.milliseconds() > 600) {

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


            /** Lift **/

            // Motor tick count is equal to 384.5

            switch (liftState) {

                case LS_STATIONARY:

                    if (gamepad2.dpad_up) {

                        liftHand = LiftHand.LH_TIPPED;

                        arm.setPosition(arm_forward_alliance);

                        liftState = LiftState.LS_EXTENDING;

                    } else if (gamepad2.dpad_left) {

                        liftHand = liftHand.LH_BALANCED;

                        arm.setPosition(arm_forward_alliance);

                        liftState = LiftState.LS_EXTENDING;

                    } else if (gamepad2.y) {

                        liftHand = LiftHand.LH_FAR;

                        arm.setPosition(arm_forward_shared);

                        liftState = LiftState.LS_EXTENDING;

                    } else if (gamepad2.b) {

                        liftHand = LiftHand.LH_CENTER;

                        arm.setPosition(arm_forward_shared);

                        liftState = LiftState.LS_EXTENDING;

                    } else if (gamepad2.a) {

                        liftHand = LiftHand.LH_NEAR;

                        arm.setPosition(arm_forward_shared);

                        liftState = LiftState.LS_EXTENDING;

                    }

                    break;

                case LS_EXTENDING:

                    // Check if button was pressed
                    if (liftHand == LiftHand.LH_TIPPED) {

                        // Run lift
                        liftLeft.setTargetPosition(TARGET_TIPPED);
                        liftRight.setTargetPosition(TARGET_TIPPED);
                        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftLeft.setPower(1.0);
                        liftRight.setPower(1.0);

                        liftState = LiftState.LS_TARGET;

                    } else if (liftHand == liftHand.LH_BALANCED) {

                        // Run lift
                        liftLeft.setTargetPosition(TARGET_BALANCED);
                        liftRight.setTargetPosition(TARGET_BALANCED);
                        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftLeft.setPower(1.0);
                        liftRight.setPower(1.0);

                        liftState = LiftState.LS_TARGET;

                    } else if (liftHand == LiftHand.LH_FAR) {

                        // Run lift
                        liftLeft.setTargetPosition(TARGET_FAR);
                        liftRight.setTargetPosition(TARGET_FAR);
                        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftLeft.setPower(1.0);
                        liftRight.setPower(1.0);

                        liftState = LiftState.LS_TURRET_DOWN;

                    } else if (liftHand == LiftHand.LH_CENTER) {

                        // Run lift
                        liftLeft.setTargetPosition(TARGET_CENTER);
                        liftRight.setTargetPosition(TARGET_CENTER);
                        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftLeft.setPower(1.0);
                        liftRight.setPower(1.0);

                        liftState = LiftState.LS_TURRET_DOWN;

                    } else if (liftHand == LiftHand.LH_NEAR) {

                        // Run lift
                        liftLeft.setTargetPosition(TARGET_NEAR);
                        liftRight.setTargetPosition(TARGET_NEAR);
                        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftLeft.setPower(1.0);
                        liftRight.setPower(1.0);

                        liftState = LiftState.LS_TURRET_DOWN;
                    }

                    break;

                case LS_TURRET_DOWN:

                    turret.setPosition(turret_target);

                    liftState = LiftState.LS_TARGET;

                    break;

                case LS_TARGET:

                    // Check if the lift has fully extended
                    if (liftHand == LiftHand.LH_TIPPED) {

                        if ((Math.abs(liftLeft.getCurrentPosition() - TARGET_TIPPED)) < 10) {

                            // Set state to depositing
                            liftState = LiftState.LS_DEPOSITING_ALLIANCE;

                        }

                    } else if (liftHand == LiftHand.LH_BALANCED) {

                        if ((Math.abs(liftLeft.getCurrentPosition() - TARGET_BALANCED)) < 10) {

                            // Set state to depositing
                            liftState = LiftState.LS_DEPOSITING_ALLIANCE;

                        }

                    } else if (liftHand == LiftHand.LH_FAR) {

                        if ((Math.abs(liftLeft.getCurrentPosition() - TARGET_FAR)) < 5) {

                            // Set state to depositing
                            liftState = LiftState.LS_DEPOSITING_SHARED;

                        }

                    } else if (liftHand == LiftHand.LH_CENTER) {

                        if ((Math.abs(liftLeft.getCurrentPosition() - TARGET_MIDDLE)) < 5) {

                            // Set state to depositing
                            liftState = LiftState.LS_DEPOSITING_SHARED;

                        }

                    } else if (liftHand == LiftHand.LH_NEAR) {

                        if ((Math.abs(liftLeft.getCurrentPosition() - TARGET_NEAR)) < 5) {

                            // Set state to depositing
                            liftState = LiftState.LS_DEPOSITING_SHARED;

                        }
                    }

                    break;

                case LS_DEPOSITING_ALLIANCE:

                    if (gamepad2.dpad_right) {

                        bucket.setPosition(bucket_right);

                        liftHand = LiftHand.LH_RETRACT_ALLIANCE;

                        liftState = LiftState.LS_RELEASED;

                    }

                    break;

                case LS_DEPOSITING_SHARED:

                    if (gamepad2.x) {

                        bucket.setPosition(bucket_left);

                        liftHand = LiftHand.LH_RETRACT_SHARED;

                        liftState = LiftState.LS_RELEASED;

                    }

                    break;

                case LS_RELEASED:

                    if (liftHand == LiftHand.LH_RETRACT_ALLIANCE) {

                        if (bucketSensor.alpha() < 60) {

                            bucket.setPosition(bucket_down);

                            // Set lift state to released
                            liftState = LiftState.LS_RETRACTING;

                        }

                    } else if (liftHand == LiftHand.LH_RETRACT_SHARED) {

                        if (bucketSensor.alpha() < 60) {

                            bucket.setPosition(bucket_down);

                            liftState = LiftState.LS_TURRET_UP;

                        }
                    }

                    break;

                case LS_TURRET_UP:

                    turretTimer.reset();

                    turret.setPosition(turret_center);

                    arm.setPosition(arm_intermediate);

                    liftState = LiftState.LS_TURRET_CHECK;

                    break;

                case LS_TURRET_CHECK:

                    if (turretTimer.milliseconds() > 2000) {

                        liftState = LiftState.LS_RETRACTING;
                    }

                    break;

                case LS_RETRACTING:

                    if ((Math.abs(liftLeft.getCurrentPosition())) > 5) {

                        // Retract lift
                        liftLeft.setTargetPosition(0);
                        liftRight.setTargetPosition(0);
                        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftLeft.setPower(-1.0);
                        liftLeft.setPower(-1.0);

                        // Set lift state to retracting
                        liftState = LiftState.LS_IDLE;

                    }

                    break;

                case LS_IDLE:

                    // Check if lift is fully retracted
                    if (Math.abs(liftLeft.getCurrentPosition()) < 5) {

                        arm.setPosition(arm_backward);

                        liftLeft.setPower(0);
                        liftRight.setPower(0);

                        // Set lift state to empty
                        liftState = LiftState.LS_STATIONARY;

                    }

                    break;
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


            if (gamepad2.right_stick_y > 0.0) {

                float e = gamepad1.right_stick_y;

                c_Left.setPower(Math.max(0, Math.min(1, (5 * Math.pow(e - 0.5, 3) + 0.7))));
                c_Right.setPower(Math.max(0, Math.min(1, (5 * Math.pow(e - 0.5, 3) + 0.7))));

            } else if (gamepad2.right_stick_y < 0.0) {

                float e = gamepad1.right_stick_y;

                c_Left.setPower(-(Math.max(0, Math.min(1, (5 * Math.pow(e - 0.5, 3) + 0.7)))));
                c_Right.setPower(-(Math.max(0, Math.min(1, (5 * Math.pow(e - 0.5, 3) + 0.7)))));

            } else {

                c_Left.setPower(0);
                c_Right.setPower(0);

            }

            // Manual Lift

            switch (manualLiftState) {

                case MLS_STATIONARY:

                    if (gamepad2.right_trigger > 0.0) {

                        manualLiftState = ManualLiftState.MLS_EXTENDING;
                    }

                    break;

                case MLS_EXTENDING:

                    float c = gamepad2.right_trigger;

                    if (c > 0.0) {

                        int d = (int) (350 * c);

                        turret.setPosition(turret_center);
                        arm.setPosition(arm_forward_alliance);

                        if (liftLeft.getCurrentPosition() != d) {

                            liftLeft.setTargetPosition(d);
                            liftRight.setTargetPosition(d);
                            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            liftLeft.setPower(1.0);
                            liftRight.setPower(1.0);
                        }

                    } else if (c == 0.0) {

                        turret.setPosition(turret_center);
                        arm.setPosition(arm_intermediate);

                        manualLiftState = ManualLiftState.MLS_RETRACTING;
                    }

                    break;

                case MLS_RETRACTING:

                    turret.setPosition(turret_center);

                    liftLeft.setTargetPosition(0);
                    liftRight.setTargetPosition(0);
                    liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftRight.setPower(-1.0);
                    liftLeft.setPower(-1.0);

                    manualLiftState = ManualLiftState.MLS_ARM_CHECK;

                    break;

                case MLS_ARM_CHECK:

                    turret.setPosition(turret_center);

                    if (liftLeft.getCurrentPosition() < 5) {

                        arm.setPosition(arm_backward);

                        manualLiftState = ManualLiftState.MLS_IDLE;
                    }

                    break;

                case MLS_IDLE:

                    liftLeft.setPower(0);
                    liftRight.setPower(0);

                    manualLiftState = ManualLiftState.MLS_STATIONARY;

                    break;
            }

            if (gamepad1.y) {

                arm.setPosition(arm_forward_alliance);
            }

            if (gamepad1.a) {

                arm.setPosition(arm_backward);
            }

            if (gamepad1.x) {

                bucket.setPosition(bucket_down);
            }

            if (gamepad1.b) {

                bucket.setPosition(bucket_right);
            }

            /** Reset Encoders **/

//            if (gamepad2.right_trigger != 0) {
//
//                liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // set motor ticks to 0
//                liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "leftFront (%.2f), leftBack (%.2f), rightFront (%.2f), rightBack (%.2f)",
                    leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
            telemetry.addData("Left Sensor: ", colorSensor_left.alpha());
            telemetry.addData("Right Sensor: ", colorSensor_right.alpha());
            telemetry.addData("BucketSensor: ", bucketSensor.alpha());
            telemetry.addData("Intake State: ", intakeState);
            telemetry.addData("C_Left: ", c_Left.getPower());
            telemetry.addData("C_Right: ", c_Right.getPower());
            telemetry.addData("Lift State: ", liftState);
            telemetry.addData("Lift Left Encoders: ", liftLeft.getCurrentPosition());
            telemetry.addData("Lift Right Encoders: ", liftRight.getCurrentPosition());
            telemetry.addData("G2 Right Trigger: ", gamepad2.right_trigger);
            telemetry.addData("Manual Lift State: ", manualLiftState);
            telemetry.addData("Joystick: ", gamepad2.right_stick_y);
            telemetry.update();
        }
    }
}