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
public class RoboDrive extends LinearOpMode {

    // 8 Motors
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor leftIntake;
    DcMotor rightIntake;
    DcMotor liftLeft;
    DcMotor liftRight;

    // 9 Servos
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
    ElapsedTime turretTimer = new ElapsedTime();
    ElapsedTime armTimer = new ElapsedTime();
    ElapsedTime bucketTimer = new ElapsedTime();
    ElapsedTime leftCaptureTimer = new ElapsedTime();
    ElapsedTime rightCaptureTimer = new ElapsedTime();
    ElapsedTime transferTimer = new ElapsedTime();
    ElapsedTime carouselTimer = new ElapsedTime();

    // Enum States
    enum IntakeState {

        IS_STATIONARY,
        IS_SETUP,
        IS_INTAKE,
        IS_CAPTURE,
        IS_TRANSFER,
        IS_OUT,
        IS_COMPLETE,
        IS_COMPLETE_SPECIAL,
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
        LS_ARM_UP,
        LS_ARM_CHECK,
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
        LS_RELEASED_SLOW_RIGHT_HALF,
        LS_RELEASED_SLOW_LEFT_HALF,
        LS_RELEASED_SLOW_RIGHT_DUMP,
        LS_RELEASED_SLOW_LEFT_DUMP,
        LS_RETRACTING,
        LS_IDLE,
        LS_RESET
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

    enum DepositState {

        DS_STATIONARY,
        DS_TURNED
    }

    enum DepositType {

        DT_NORMAL,
        DT_SLOW
    }

    enum DepositHand {

        DH_STATIONARY,
        DH_LEFT,
        DH_RIGHT
    }

    enum CarouselState {

        CS_STATIONARY,
        CS_TURNING
    }

    enum CarouselHand {

        CH_LEFT,
        CH_RIGHT
    }

    enum CheckDepositButtonState {

        CDBS_STATIONARY,
        CDBS_PRESSED
    }

    enum RobotState {

        RS_AUTO,
        RS_MANUAL
    }

    enum FieldSide {

        FS_BLUE,
        FS_RED
    }

    enum CheckRobotButtonState {

        CRBS_STATIONARY,
        CRBS_PRESSED
    }

    // State Declarations
    IntakeState intakeState = IntakeState.IS_STATIONARY;
    IntakeHand intakeHand = IntakeHand.IH_NEITHER;
    BucketHand bucketHand = BucketHand.BH_CENTER;
    LiftState liftState = LiftState.LS_STATIONARY;
    LiftHand liftHand = LiftHand.LH_IDLE;
    ManualLiftState manualLiftState = ManualLiftState.MLS_STATIONARY;
    DepositState depositState = DepositState.DS_STATIONARY;
    DepositType depositType = DepositType.DT_NORMAL;
    DepositHand depositHand = DepositHand.DH_STATIONARY;
    CarouselState carouselState = CarouselState.CS_STATIONARY;
    CarouselHand carouselHand = CarouselHand.CH_LEFT;
    CheckDepositButtonState checkDepositButtonState = CheckDepositButtonState.CDBS_STATIONARY;
    RobotState robotState = RobotState.RS_AUTO;
    CheckRobotButtonState checkRobotButtonState = CheckRobotButtonState.CRBS_STATIONARY;
    FieldSide fieldSide = FieldSide.FS_BLUE;

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
        turret = hardwareMap.servo.get("turret");
        c_Left = hardwareMap.crservo.get("c_Left");
        c_Right = hardwareMap.crservo.get("c_Right");

        // Color sensors
        colorSensor_left = hardwareMap.get(ColorRangeSensor.class, "colorSensor_left");
        colorSensor_right = hardwareMap.get(ColorRangeSensor.class, "colorSensor_right");
        bucketSensor = hardwareMap.get(ColorRangeSensor.class, "bucketSensor");

        double i_minRange_topLeft = 0.15;   // 0.08
        double i_maxRange_topLeft = 0.92;
        double i_minRange_bottomLeft = 0.90;   // 0.92
        double i_maxRange_bottomLeft = 0.13;
        double i_minRange_topRight = 0.87;   // 0.94
        double i_maxRange_topRight = 0.16;
        double i_minRange_bottomRight = 0.16;   // 0.08
        double i_maxRange_bottomRight = 0.87;

        // Intake
        double highSweepPower = 0.8;

        // Deposit servo positions
        double bucket_down = 0.48;
        double bucket_left = 0.20;
        double bucket_right = 0.76;
        double arm_forward_alliance = 0.35;
        double arm_forward_shared = 0.10;
        double arm_forward_middle = 0.15;
        double arm_forward_manual = 0.20;
        double arm_intermediate = 0.70;
        double arm_backward = 0.90;  // 0.92
        double turret_center = 0.51;
        double turret_targetBlue = 0.35;
        double turret_targetRed = 0.61;

        // Initialize deposit positions
        bucket.setPosition(bucket_down);
        arm.setPosition(arm_backward);
        turret.setPosition(turret_center); 

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

            switch (intakeState) {

                case IS_STATIONARY:

                    if (bucketSensor.alpha() > 70) {

                        intakeState = IntakeState.IS_COMPLETE_SPECIAL;

                    } else if (gamepad1.left_bumper) {

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

                    if (gamepad1.left_trigger != 0 && intakeHand == IntakeHand.IH_LEFT) {

                        intakeState = IntakeState.IS_RESET_L;

                    } else if (gamepad1.right_trigger != 0 && intakeHand == IntakeHand.IH_RIGHT) {

                        intakeState = IntakeState.IS_RESET_R;

                    } else if (intakeHand == IntakeHand.IH_LEFT) {

                        bucket.setPosition(bucket_left);

                        if (colorSensor_left.alpha() > 2000) {

                            // left intake flips up
                            i_topLeft.setPosition(i_maxRange_topLeft);
                            i_bottomLeft.setPosition(i_maxRange_bottomLeft);

                            leftCaptureTimer.reset();

                            intakeState = IntakeState.IS_CAPTURE;

                        } else if (gamepad1.right_bumper) {

                            intakeHand = IntakeHand.IH_RIGHT;
                            intakeState = IntakeState.IS_SETUP;
                        }

                    } else if (intakeHand == IntakeHand.IH_RIGHT) {

                        bucket.setPosition(bucket_right);

                        if (colorSensor_right.alpha() > 2000) {

                            // left intake flips up
                            i_topRight.setPosition(i_maxRange_topRight);
                            i_bottomRight.setPosition(i_maxRange_bottomRight);

                            rightCaptureTimer.reset();

                            intakeState = IntakeState.IS_CAPTURE;

                        } else if (gamepad1.left_bumper) {

                            intakeHand = IntakeHand.IH_LEFT;
                            intakeState = IntakeState.IS_SETUP;
                        }
                    }

                    break;

                case IS_CAPTURE:

                    if (intakeHand == IntakeHand.IH_LEFT) {

                        // hold elements in intake
                        leftIntake.setPower(0.25);

                        if (leftCaptureTimer.milliseconds() > 1000) {

                            transferTimer.reset();
                            intakeState = IntakeState.IS_TRANSFER;

                        }

                    } else if (intakeHand == IntakeHand.IH_RIGHT) {

                        // hold elements in intake
                        rightIntake.setPower(0.25);

                        if (rightCaptureTimer.milliseconds() > 1000) {

                            transferTimer.reset();
                            intakeState = IntakeState.IS_TRANSFER;

                        }
                    }

                    break;

                case IS_TRANSFER:

                    if (intakeHand == IntakeHand.IH_LEFT) {

                        leftIntake.setPower(-highSweepPower);

                        if (transferTimer.milliseconds() > 250) {

                            leftIntakeTimer.reset();
                            intakeState = IntakeState.IS_OUT;

                        }

                    } else if (intakeHand == IntakeHand.IH_RIGHT) {

                        rightIntake.setPower(-highSweepPower);

                        if (transferTimer.milliseconds() > 250) {

                            rightIntakeTimer.reset();
                            intakeState = IntakeState.IS_OUT;

                        }
                    }

                    break;

                case IS_OUT:

                    if (intakeHand == IntakeHand.IH_LEFT) {

                        if (leftIntakeTimer.milliseconds() > 500) {

                            if (bucketSensor.alpha() > 200) {

                                bucket.setPosition(bucket_down);
                                leftIntake.setPower(0);
                                intakeState = IntakeState.IS_COMPLETE;

                            } else if (colorSensor_left.alpha() < 2000) {

                                bucket.setPosition(bucket_down);
                                leftIntake.setPower(0);
                                intakeState = IntakeState.IS_SETUP;
                            }

                        }

                    } else if (intakeHand == IntakeHand.IH_RIGHT) {

                        if (rightIntakeTimer.milliseconds() > 500) {

                            if (bucketSensor.alpha() > 200) {

                                bucket.setPosition(bucket_down);
                                rightIntake.setPower(0);
                                intakeState = IntakeState.IS_COMPLETE;

                            } else if (colorSensor_right.alpha() < 2000) {

                                bucket.setPosition(bucket_down);
                                rightIntake.setPower(0);
                                intakeState = IntakeState.IS_SETUP;
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

                case IS_COMPLETE_SPECIAL:

                    if (bucketSensor.alpha() < 70) {

                        intakeState = IntakeState.IS_STATIONARY;
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

            switch (depositState) {

                case DS_STATIONARY:

                    bucketTimer.reset();

                    if (gamepad1.x) {

                        depositHand = DepositHand.DH_LEFT;

                        bucket.setPosition(bucket_left);

                        depositState = DepositState.DS_TURNED;

                    } else if (gamepad1.b) {

                        depositHand = DepositHand.DH_RIGHT;

                        bucket.setPosition(bucket_right);

                        depositState = DepositState.DS_TURNED;
                    }

                    break;

                case DS_TURNED:

                    if (depositHand == DepositHand.DH_LEFT) {

                        if (bucketTimer.milliseconds() > 500) {

                            bucket.setPosition(bucket_down);
                            arm.setPosition(arm_backward);
                            depositHand = DepositHand.DH_STATIONARY;
                            depositState = DepositState.DS_STATIONARY;
                        }

                    } else if (depositHand == DepositHand.DH_RIGHT) {

                        if (bucketTimer.milliseconds() > 500) {

                            bucket.setPosition(bucket_down);
                            arm.setPosition(arm_backward);
                            depositHand = DepositHand.DH_STATIONARY;
                            depositState = DepositState.DS_STATIONARY;
                        }
                    }

                    break;
            }

            if (gamepad1.y) {

                arm.setPosition(arm_forward_manual);
            }

            if (gamepad1.a) {

                arm.setPosition(arm_backward);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "leftFront (%.2f), leftBack (%.2f), rightFront (%.2f), rightBack (%.2f)",
                    leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
            telemetry.update();
        }
    }
}