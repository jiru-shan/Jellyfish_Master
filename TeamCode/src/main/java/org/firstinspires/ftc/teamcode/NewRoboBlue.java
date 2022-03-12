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
import org.firstinspires.ftc.teamcode.past_code.Deposit;
import org.firstinspires.ftc.teamcode.past_code.Lift;

import java.util.List;

@Config
@TeleOp
public class NewRoboBlue extends LinearOpMode {

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

    enum ObjectType {

        OT_CUBE,
        OT_BALL
    }

//    enum RobotState {
//
//        RS_AUTO,
//        RS_MANUAL
//    }
//
//    enum CheckRobotButtonState {
//
//        CRBS_STATIONARY,
//        CRBS_PRESSED
//    }

    enum FieldSide {

        FS_BLUE,
        FS_RED
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
    FieldSide fieldSide = FieldSide.FS_BLUE;
    ObjectType objectType = ObjectType.OT_CUBE;

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
        double bucket_left = 0.17;
        double bucket_right = 0.81;
        double arm_forward_alliance = 0.35;
        double arm_forward_middle = 0.15;
        double arm_forward_shared = 0.10;
        double arm_intermediate = 0.70;
        double arm_backward = 0.90;  // 0.92
        double turret_center = 0.51;
        double turret_targetBlue = 0.35;
        double turret_targetRed = 0.61;

        // Servo positions
        double i_minRange_topLeft = 0.15;
        double i_maxRange_topLeft = 0.96;
        double i_minRange_bottomLeft = 0.90;
        double i_maxRange_bottomLeft = 0.09;
        double i_minRange_topRight = 0.90;
        double i_maxRange_topRight = 0.09;
        double i_minRange_bottomRight = 0.16;
        double i_maxRange_bottomRight = 0.97;

        // Lift Positions
        int TARGET_TIPPED = 450;
        int TARGET_BALANCED = 415;
        int TARGET_MIDDLE = 300;
        int TARGET_NEAR = 50;
        int TARGET_CENTER = 130;
        int TARGET_FAR = 210;

        // Reset encoders
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // set motor ticks to 0
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Prevent drift
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize deposit positions
        bucket.setPosition(bucket_down);
        arm.setPosition(arm_backward);
        turret.setPosition(turret_center);

        // Initialize intake positions
        i_topLeft.setPosition(i_maxRange_topLeft);
        i_bottomLeft.setPosition(i_maxRange_bottomLeft);
        i_topRight.setPosition(i_maxRange_topRight);
        i_bottomRight.setPosition(i_maxRange_bottomRight);

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

            switch (intakeState) {

                case IS_STATIONARY:

                    if (bucketSensor.alpha() > 70) {

                        intakeState = IntakeState.IS_COMPLETE_SPECIAL;

                    } else if (fieldSide == FieldSide.FS_BLUE && gamepad1.left_bumper) {

                        intakeHand = IntakeHand.IH_LEFT;

                        intakeState = IntakeState.IS_SETUP;

                    } else if (fieldSide == FieldSide.FS_RED && gamepad1.left_bumper) {

                        intakeHand = IntakeHand.IH_RIGHT;

                        intakeState = IntakeState.IS_SETUP;

                    } else if (fieldSide == FieldSide.FS_BLUE && gamepad1.right_bumper) {

                        intakeHand = IntakeHand.IH_RIGHT;

                        intakeState = IntakeState.IS_SETUP;

                    } else if (fieldSide == FieldSide.FS_RED && gamepad1.right_bumper) {

                        intakeHand = IntakeHand.IH_LEFT;

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

                        if (liftLeft.getCurrentPosition() < 5) {

                            // turn bucket left
                            bucket.setPosition(bucket_left);

                        } else {

                            intakeState = IntakeState.IS_SETUP;
                        }

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

                        if (liftLeft.getCurrentPosition() < 5) {

                            // turn bucket left
                            bucket.setPosition(bucket_right);

                        } else {

                            intakeState = IntakeState.IS_SETUP;
                        }

                        bucketHand = BucketHand.BH_RIGHT;

                        arm.setPosition(arm_backward);
                        turret.setPosition(turret_center);

                        intakeState = IntakeState.IS_INTAKE;
                    }

                    break;

                case IS_INTAKE:

                    if (fieldSide == FieldSide.FS_BLUE && gamepad1.left_trigger != 0 && intakeHand == IntakeHand.IH_LEFT) {

                        intakeState = IntakeState.IS_RESET_L;

                    } else if (fieldSide == FieldSide.FS_BLUE && gamepad1.right_trigger != 0 && intakeHand == IntakeHand.IH_RIGHT) {

                        intakeState = IntakeState.IS_RESET_R;

                    } else if (fieldSide == FieldSide.FS_RED && gamepad1.right_trigger != 0 && intakeHand == IntakeHand.IH_LEFT) {

                        intakeState = IntakeState.IS_RESET_L;

                    } else if (fieldSide == FieldSide.FS_RED && gamepad1.left_trigger != 0 && intakeHand == IntakeHand.IH_RIGHT) {

                        intakeState = IntakeState.IS_RESET_R;

                    } else if (intakeHand == IntakeHand.IH_LEFT) {

                        if (liftLeft.getCurrentPosition() < 5) {

                            bucket.setPosition(bucket_left);

                        } else {

                            intakeState = IntakeState.IS_INTAKE;
                        }

                        if (colorSensor_left.alpha() > 800) {

                            // left intake flips up
                            i_topLeft.setPosition(i_maxRange_topLeft);
                            i_bottomLeft.setPosition(i_maxRange_bottomLeft);

                            leftCaptureTimer.reset();

                            if (colorSensor_left.blue() > 1000) {

                                objectType = ObjectType.OT_BALL;

                                gamepad1.rumble(100);
                                gamepad2.rumble(100);

                            } else {

                                objectType = ObjectType.OT_CUBE;
                            }

                            intakeState = IntakeState.IS_CAPTURE;

                        } else if (fieldSide == FieldSide.FS_BLUE && gamepad1.right_bumper) {

                            intakeHand = IntakeHand.IH_RIGHT;
                            intakeState = IntakeState.IS_SETUP;

                        } else if (fieldSide == FieldSide.FS_RED && gamepad1.left_bumper) {

                            intakeHand = IntakeHand.IH_RIGHT;
                            intakeState = IntakeState.IS_SETUP;
                        }

                    } else if (intakeHand == IntakeHand.IH_RIGHT) {

                        if (liftLeft.getCurrentPosition() < 5) {

                            bucket.setPosition(bucket_right);

                        } else {

                            intakeState = IntakeState.IS_INTAKE;
                        }

                        if (colorSensor_right.alpha() > 800) {

                            // left intake flips up
                            i_topRight.setPosition(i_maxRange_topRight);
                            i_bottomRight.setPosition(i_maxRange_bottomRight);

                            rightCaptureTimer.reset();

                            if (colorSensor_right.blue() > 1000) {

                                objectType = ObjectType.OT_BALL;

                                gamepad1.rumble(100);
                                gamepad2.rumble(100);

                            } else {

                                objectType = ObjectType.OT_CUBE;
                            }

                            intakeState = IntakeState.IS_CAPTURE;

                        } else if (fieldSide == FieldSide.FS_BLUE && gamepad1.left_bumper) {

                            intakeHand = IntakeHand.IH_LEFT;
                            intakeState = IntakeState.IS_SETUP;

                        } else if (fieldSide == FieldSide.FS_RED & gamepad1.right_bumper) {

                            intakeHand = IntakeHand.IH_LEFT;
                            intakeState = IntakeState.IS_SETUP;
                        }
                    }

                    break;

                case IS_CAPTURE:

                    if (intakeHand == IntakeHand.IH_LEFT) {

                        bucket.setPosition(bucket_left);

                        // hold elements in intake
                        leftIntake.setPower(0.25);

                        if (leftCaptureTimer.milliseconds() > 800) {

                            transferTimer.reset();
                            intakeState = IntakeState.IS_TRANSFER;

                        }

                    } else if (intakeHand == IntakeHand.IH_RIGHT) {

                        bucket.setPosition(bucket_right);

                        // hold elements in intake
                        rightIntake.setPower(0.25);

                        if (rightCaptureTimer.milliseconds() > 800) {

                            transferTimer.reset();
                            intakeState = IntakeState.IS_TRANSFER;

                        }
                    }

                    break;

                case IS_TRANSFER:

                    if (intakeHand == IntakeHand.IH_LEFT) {

                        leftIntake.setPower(-highSweepPower);

                        if (transferTimer.milliseconds() > 250 || bucketSensor.alpha() > 200) {

                            leftIntakeTimer.reset();
                            intakeState = IntakeState.IS_OUT;

                        }

                    } else if (intakeHand == IntakeHand.IH_RIGHT) {

                        rightIntake.setPower(-highSweepPower);

                        if (transferTimer.milliseconds() > 250 || bucketSensor.alpha() > 200) {

                            rightIntakeTimer.reset();
                            intakeState = IntakeState.IS_OUT;

                        }
                    }

                    break;

                case IS_OUT:

                    if (bucketSensor.alpha() > 200)

                        bucket.setPosition(bucket_down); // p sure this line actually does nothing but imma just add it here

                    if (intakeHand == IntakeHand.IH_LEFT) {

                        if (leftIntakeTimer.milliseconds() > 500 || bucketSensor.alpha() > 200) {

                            if (bucketSensor.alpha() > 200) {

                                bucket.setPosition(bucket_down);
                                leftIntake.setPower(0);
                                intakeState = IntakeState.IS_COMPLETE;

                            } else if (colorSensor_left.alpha() < 1200) {

                                bucket.setPosition(bucket_down);
                                leftIntake.setPower(0);
                                intakeState = IntakeState.IS_SETUP;
                            }

                        }

                    } else if (intakeHand == IntakeHand.IH_RIGHT) {

                        if (rightIntakeTimer.milliseconds() > 500 || bucketSensor.alpha() > 200) {

                            if (bucketSensor.alpha() > 200) {

                                bucket.setPosition(bucket_down);
                                rightIntake.setPower(0);
                                intakeState = IntakeState.IS_COMPLETE;

                            } else if (colorSensor_right.alpha() < 1200) {

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

                    } else if (gamepad2.dpad_down) {

                        liftHand = liftHand.LH_MIDDLE;

                        arm.setPosition(arm_forward_middle);

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

                    } else if (liftHand == LiftHand.LH_MIDDLE) {

                        // Run lift
                        liftLeft.setTargetPosition(TARGET_MIDDLE);
                        liftRight.setTargetPosition(TARGET_MIDDLE);
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

                    if (fieldSide == FieldSide.FS_BLUE) {

                        turret.setPosition(turret_targetBlue);

                    } else if (fieldSide == FieldSide.FS_RED) {

                        turret.setPosition(turret_targetRed);
                    }

                    liftState = LiftState.LS_TARGET;

                    break;

                case LS_TARGET:

                    int adjust = 50;

                    if (gamepad2.right_trigger != 0) {

                        liftLeft.setTargetPosition(liftLeft.getCurrentPosition() + adjust);
                        liftRight.setTargetPosition(liftRight.getCurrentPosition() + adjust);
                        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftLeft.setPower(1.0);
                        liftRight.setPower(1.0);

                    } else if (gamepad2.left_trigger != 0) {

                        liftLeft.setTargetPosition(liftLeft.getCurrentPosition() - adjust);
                        liftRight.setTargetPosition(liftRight.getCurrentPosition() - adjust);
                        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftLeft.setPower(-1.0);
                        liftRight.setPower(-1.0);

                    } else if (gamepad1.y) {

                        bucketTimer.reset();

                        bucket.setPosition(bucket_right);

                        // Check if the lift has fully extended
                        if (liftHand == LiftHand.LH_TIPPED) {

                            // Set state to depositing
                            liftHand = LiftHand.LH_RETRACT_ALLIANCE;

                        } else if (liftHand == LiftHand.LH_BALANCED) {

                            // Set state to depositing
                            liftHand = LiftHand.LH_RETRACT_ALLIANCE;

                        } else if (liftHand == LiftHand.LH_MIDDLE) {

                            // Set state to depositing
                            liftHand = LiftHand.LH_RETRACT_ALLIANCE;

                        } else if (liftHand == LiftHand.LH_FAR) {

                            // Set state to depositing
                            liftHand = LiftHand.LH_RETRACT_SHARED;

                        } else if (liftHand == LiftHand.LH_CENTER) {

                            // Set state to depositing
                            liftHand = LiftHand.LH_RETRACT_SHARED;

                        } else if (liftHand == LiftHand.LH_NEAR) {

                            // Set state to depositing
                            liftHand = LiftHand.LH_RETRACT_SHARED;
                        }

                        liftState = LiftState.LS_RELEASED;

                    } else if (gamepad1.b) {

                        bucketTimer.reset();

                        if (objectType == ObjectType.OT_CUBE) {

                            bucket.setPosition(0.65);

                        } else if (objectType == ObjectType.OT_BALL) {

                            bucket.setPosition(0.68);
                        }

                        // Check if the lift has fully extended
                        if (liftHand == LiftHand.LH_TIPPED) {

                            // Set state to depositing
                            liftHand = LiftHand.LH_RETRACT_ALLIANCE;

                        } else if (liftHand == LiftHand.LH_BALANCED) {

                            // Set state to depositing
                            liftHand = LiftHand.LH_RETRACT_ALLIANCE;

                        } else if (liftHand == LiftHand.LH_MIDDLE) {

                            // Set state to depositing
                            liftHand = LiftHand.LH_RETRACT_ALLIANCE;

                        } else if (liftHand == LiftHand.LH_FAR) {

                            // Set state to depositing
                            liftHand = LiftHand.LH_RETRACT_SHARED;

                        } else if (liftHand == LiftHand.LH_CENTER) {

                            // Set state to depositing
                            liftHand = LiftHand.LH_RETRACT_SHARED;


                        } else if (liftHand == LiftHand.LH_NEAR) {

                            // Set state to depositing
                            liftHand = LiftHand.LH_RETRACT_SHARED;
                        }

                        liftState = LiftState.LS_RELEASED_SLOW_RIGHT_HALF;

                    } else if (gamepad1.x) {

                        bucketTimer.reset();

                        bucket.setPosition(bucket_left);

                        // Check if the lift has fully extended
                        if (liftHand == LiftHand.LH_TIPPED) {

                            // Set state to depositing
                            liftHand = LiftHand.LH_RETRACT_ALLIANCE;

                        } else if (liftHand == LiftHand.LH_BALANCED) {

                            // Set state to depositing
                            liftHand = LiftHand.LH_RETRACT_ALLIANCE;

                        } else if (liftHand == LiftHand.LH_MIDDLE) {

                            // Set state to depositing
                            liftHand = LiftHand.LH_RETRACT_ALLIANCE;

                        } else if (liftHand == LiftHand.LH_FAR) {

                            // Set state to depositing
                            liftHand = LiftHand.LH_RETRACT_SHARED;

                        } else if (liftHand == LiftHand.LH_CENTER) {

                            // Set state to depositing
                            liftHand = LiftHand.LH_RETRACT_SHARED;


                        } else if (liftHand == LiftHand.LH_NEAR) {

                            // Set state to depositing
                            liftHand = LiftHand.LH_RETRACT_SHARED;
                        }

                        liftState = LiftState.LS_RELEASED;

                    } else if (gamepad1.a) {

                        bucketTimer.reset();

                        if (objectType == ObjectType.OT_CUBE) {

                            bucket.setPosition(0.31);

                        } else if (objectType == ObjectType.OT_BALL) {

                            bucket.setPosition(0.28);
                        }

                        // Check if the lift has fully extended
                        if (liftHand == LiftHand.LH_TIPPED) {

                            // Set state to depositing
                            liftHand = LiftHand.LH_RETRACT_ALLIANCE;

                        } else if (liftHand == LiftHand.LH_BALANCED) {

                            // Set state to depositing
                            liftHand = LiftHand.LH_RETRACT_ALLIANCE;

                        } else if (liftHand == LiftHand.LH_MIDDLE) {

                            // Set state to depositing
                            liftHand = LiftHand.LH_RETRACT_ALLIANCE;

                        } else if (liftHand == LiftHand.LH_FAR) {

                            // Set state to depositing
                            liftHand = LiftHand.LH_RETRACT_SHARED;

                        } else if (liftHand == LiftHand.LH_CENTER) {

                            // Set state to depositing
                            liftHand = LiftHand.LH_RETRACT_SHARED;

                        } else if (liftHand == LiftHand.LH_NEAR) {

                            // Set state to depositing
                            liftHand = LiftHand.LH_RETRACT_SHARED;
                        }

                        liftState = LiftState.LS_RELEASED_SLOW_LEFT_HALF;
                    }

                    break;


                case LS_RELEASED_SLOW_RIGHT_HALF:

                    if (bucketTimer.milliseconds() > 250) {

                        bucket.setPosition(bucket_right);

                        liftState = LiftState.LS_RELEASED_SLOW_RIGHT_DUMP;

                    }

                    break;

                case LS_RELEASED_SLOW_RIGHT_DUMP:

                    if (bucketTimer.milliseconds() > 500) {

                        bucket.setPosition(bucket_down);

                        liftState = LiftState.LS_RETRACTING;
                    }

                    break;

                case LS_RELEASED_SLOW_LEFT_HALF:

                    if (bucketTimer.milliseconds() > 250) {

                        bucket.setPosition(bucket_left);

                        liftState = LiftState.LS_RELEASED_SLOW_LEFT_DUMP;

                    }

                    break;

                case LS_RELEASED_SLOW_LEFT_DUMP:

                    if (bucketTimer.milliseconds() > 500) {

                        bucket.setPosition(bucket_down);

                        liftState = LiftState.LS_RETRACTING;
                    }

                    break;

                case LS_RELEASED:

                    if (liftHand == LiftHand.LH_RETRACT_ALLIANCE) {

                        if (bucketTimer.milliseconds() > 250) {

                            bucket.setPosition(bucket_down);

                            // Set lift state to released
                            liftState = LiftState.LS_RETRACTING;

                        }

                    } else if (liftHand == LiftHand.LH_RETRACT_SHARED) {

                        if (bucketTimer.milliseconds() > 250) {

                            bucket.setPosition(bucket_down);

                            liftState = LiftState.LS_ARM_UP;

                        }
                    }

                    break;

                case LS_ARM_UP:

                    armTimer.reset();

                    arm.setPosition(arm_intermediate);

                    liftState = LiftState.LS_ARM_CHECK;

                    break;

                case LS_ARM_CHECK:

                    if (turretTimer.milliseconds() > 250) {

                        liftState = LiftState.LS_TURRET_UP;
                    }

                    break;

                case LS_TURRET_UP:

                    turretTimer.reset();

                    turret.setPosition(turret_center);

                    liftState = LiftState.LS_TURRET_CHECK;

                    break;

                case LS_TURRET_CHECK:

                    if (turretTimer.milliseconds() > 250) {

                        liftState = LiftState.LS_RETRACTING;
                    }

                    break;

                case LS_RETRACTING:

                    if ((Math.abs(liftLeft.getCurrentPosition())) > 5) {

                        turret.setPosition(turret_center);

                        // Retract lift
                        liftLeft.setTargetPosition(0);
                        liftRight.setTargetPosition(0);
                        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        liftLeft.setPower(-1.0);
                        liftRight.setPower(-1.0);

                        // Set lift state to retracting
                        liftState = LiftState.LS_IDLE;

                    }

                    break;

                case LS_IDLE:

                    // Check if lift is fully retracted
                    if (Math.abs(liftLeft.getCurrentPosition()) < 5) {

                        turret.setPosition(turret_center);
                        arm.setPosition(arm_backward);

                        // Set lift state to empty
                        liftState = LiftState.LS_RESET;

                    }

                    break;

                case LS_RESET:

                    if (gamepad2.touchpad || gamepad2.x) {

                        turret.setPosition(turret_center);
                        arm.setPosition(arm_backward);
                        bucket.setPosition(bucket_down);

                        liftLeft.setPower(0);
                        liftRight.setPower(0);

                        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // set motor ticks to 0
                        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        liftState = LiftState.LS_STATIONARY;
                    }
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

                        int d = (int) (450 * c);

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

                    if (liftLeft.getCurrentPosition() < 10) {

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

//            switch (depositState) {
//
//                case DS_STATIONARY:
//
//                    bucketTimer.reset();
//
//                    if (gamepad1.x) {
//
//                        depositHand = DepositHand.DH_LEFT;
//
//                        bucket.setPosition(bucket_left);
//
//                        depositState = DepositState.DS_TURNED;
//
//                    } else if (gamepad1.b) {
//
//                        depositHand = DepositHand.DH_RIGHT;
//
//                        bucket.setPosition(bucket_right);
//
//                        depositState = DepositState.DS_TURNED;
//                    }
//
//                    break;
//
//                case DS_TURNED:
//
//                    if (depositHand == DepositHand.DH_LEFT) {
//
//                        if (bucketTimer.milliseconds() > 500) {
//
//                            bucket.setPosition(bucket_down);
//                            depositHand = DepositHand.DH_STATIONARY;
//                            depositState = DepositState.DS_STATIONARY;
//                        }
//
//                    } else if (depositHand == DepositHand.DH_RIGHT) {
//
//                        if (bucketTimer.milliseconds() > 500) {
//
//                            bucket.setPosition(bucket_down);
//                            depositHand = DepositHand.DH_STATIONARY;
//                            depositState = DepositState.DS_STATIONARY;
//                        }
//                    }
//
//                    break;
//            }

            float g = (float) (((gamepad2.left_stick_y) + 1.0) / 2.0);

            if (gamepad2.left_stick_y != 0) {

                turret.setPosition(g);

            }

            /** Drivetrain **/

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

            float a = gamepad2.right_stick_y;

            if (gamepad2.right_stick_y > 0) {

                c_Left.setPower(a);
                c_Right.setPower(a);

            } else if (gamepad2.right_stick_y < 0) {

                c_Left.setPower(a);
                c_Right.setPower(a);

            } else {

                c_Left.setPower(0);
                c_Right.setPower(0);
            }

            /** Carousel **/

            int firstInterval = 500;
            int secondInterval = 600;
            int thirdInterval = 1250;

            switch (carouselState) {

                case CS_STATIONARY:

                    if (gamepad2.left_bumper) {

                        carouselTimer.reset();
                        carouselHand = CarouselHand.CH_LEFT;
                        carouselState = CarouselState.CS_TURNING;

                    } else if (gamepad2.right_bumper) {

                        carouselTimer.reset();
                        carouselHand = CarouselHand.CH_RIGHT;
                        carouselState = CarouselState.CS_TURNING;
                    }

                    break;

                case CS_TURNING:

                    if (carouselHand == CarouselHand.CH_LEFT) {

                        if (carouselTimer.milliseconds() <= firstInterval) {

                            c_Left.setPower(0.35);
                            c_Right.setPower(0.35);

                        } else if (carouselTimer.milliseconds() > firstInterval && carouselTimer.milliseconds() <= secondInterval) {

                            c_Left.setPower((0.003 * carouselTimer.milliseconds()) - 1.25);
                            c_Right.setPower((0.003 * carouselTimer.milliseconds()) - 1.25);

                        } else if (carouselTimer.milliseconds() > secondInterval && carouselTimer.milliseconds() <= thirdInterval) {

                            c_Left.setPower(1.0);
                            c_Right.setPower(1.0);

                        } else if (carouselTimer.milliseconds() > thirdInterval) {

                            c_Left.setPower(0);
                            c_Right.setPower(0);
                            carouselState = CarouselState.CS_STATIONARY;
                        }

                    } else if (carouselHand == CarouselHand.CH_RIGHT) {

                        if (carouselTimer.milliseconds() <= firstInterval) {

                            c_Left.setPower(-0.35);
                            c_Right.setPower(-0.35);

                        } else if (carouselTimer.milliseconds() > firstInterval && carouselTimer.milliseconds() <= secondInterval) {

                            c_Left.setPower(-((0.003 * carouselTimer.milliseconds()) - 1.25));
                            c_Right.setPower(-((0.003 * carouselTimer.milliseconds()) - 1.25));

                        } else if (carouselTimer.milliseconds() > secondInterval && carouselTimer.milliseconds() <= thirdInterval) {

                            c_Left.setPower(-1.0);
                            c_Right.setPower(-1.0);

                        } else if (carouselTimer.milliseconds() > thirdInterval) {

                            c_Left.setPower(0);
                            c_Right.setPower(0);
                            carouselState = CarouselState.CS_STATIONARY;
                        }
                    }

                    break;
            }

            if (gamepad2.dpad_right) {

                turret.setPosition(turret_center);

                // Retract lift
                liftLeft.setTargetPosition(0);
                liftRight.setTargetPosition(0);
                liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftLeft.setPower(-1.0);
                liftRight.setPower(-1.0);

                // Set lift state to retracting
                liftState = LiftState.LS_IDLE;
            }

            if (gamepad2.touchpad || gamepad2.x) {

                liftLeft.setPower(0);
                liftRight.setPower(0);

                liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // set motor ticks to 0
                liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                liftState = LiftState.LS_STATIONARY;

                turret.setPosition(turret_center);
                arm.setPosition(arm_backward);
                bucket.setPosition(bucket_down);
            }



            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "leftFront (%.2f), leftBack (%.2f), rightFront (%.2f), rightBack (%.2f)",
                    leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
            telemetry.addData("Left Sensor: ", colorSensor_left.alpha());
            telemetry.addData("Right Sensor: ", colorSensor_right.alpha());
            telemetry.addData("Dist - Left", colorSensor_left.getDistance(DistanceUnit.CM));
            telemetry.addData("Dist - Right", colorSensor_right.getDistance(DistanceUnit.CM));
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
            telemetry.addData("Turret: ", turret.getPosition());
            telemetry.addData("Deposit Speed: ", checkDepositButtonState);
            telemetry.addData("Bucket Position: ", bucket.getPosition());
            telemetry.addData("TopRight: ", i_topRight.getPosition());
            telemetry.addData("BottomRight: ", i_bottomRight.getPosition());
            telemetry.addData("Blue Values - L: ", colorSensor_left.blue());
            telemetry.addData("Blue Values - R: ", colorSensor_right.blue());
            telemetry.update();
        }
    }
}