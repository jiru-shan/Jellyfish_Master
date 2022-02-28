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
public class AllianceHubTesting extends LinearOpMode {

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
    ElapsedTime strategicDepositTimer = new ElapsedTime();

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
        double arm_forward_alliance = 0.35;
        double arm_forward_shared = 0.10;
        double arm_forward_manual = 0.20;
        double arm_intermediate = 0.70;
        double arm_backward = 0.91;  // 0.92
        double turret_center = 0.51;
        double turret_targetBlue = 0.35;
        double turret_targetRed = 0.61;

        // Servo positions
        double i_minRange_topLeft = 0.15;
        double i_maxRange_topLeft = 0.96;
        double i_minRange_bottomLeft = 0.90;
        double i_maxRange_bottomLeft = 0.09;
        double i_minRange_topRight = 0.91; // 0.95
        double i_maxRange_topRight = 0.10;  // 0.08
        double i_minRange_bottomRight = 0.15; // 0.09
        double i_maxRange_bottomRight = 0.96; //0.96

        // Lift Positions
        int TARGET_TIPPED = 400;
        int TARGET_BALANCED = 320;
        int TARGET_MIDDLE = 210;
        int TARGET_CENTER = 150;
        int TARGET_FAR = 180;

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
//        i_topLeft.setPosition(i_maxRange_topLeft);
//        i_bottomLeft.setPosition(i_maxRange_bottomLeft);
//        i_topRight.setPosition(i_maxRange_topRight);
//        i_bottomRight.setPosition(i_maxRange_bottomRight);

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

            if (robotState == RobotState.RS_AUTO) {

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

                        if (gamepad1.left_trigger != 0) {

                            intakeState = IntakeState.IS_RESET_L;

                        } else if (gamepad1.right_trigger != 0) {

                            intakeState = IntakeState.IS_RESET_R;

                        } else if (intakeHand == IntakeHand.IH_LEFT) {

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

                                    leftIntake.setPower(0);
                                    intakeState = IntakeState.IS_COMPLETE;

                                } else if (colorSensor_left.alpha() < 2000) {

                                    leftIntake.setPower(0);
                                    intakeState = IntakeState.IS_SETUP;
                                }

                            }

                        } else if (intakeHand == IntakeHand.IH_RIGHT) {

                            if (rightIntakeTimer.milliseconds() > 500) {

                                if (bucketSensor.alpha() > 200) {

                                    rightIntake.setPower(0);
                                    intakeState = IntakeState.IS_COMPLETE;

                                } else if (colorSensor_right.alpha() < 2000) {

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

                            if ((Math.abs(liftLeft.getCurrentPosition() - TARGET_CENTER)) < 5) {

                                // Set state to depositing
                                liftState = LiftState.LS_DEPOSITING_SHARED;

                            }

                        } else if (liftHand == LiftHand.LH_NEAR) {

                            // Set state to depositing
                            liftState = LiftState.LS_DEPOSITING_SHARED;
                        }

                        break;

                    case LS_DEPOSITING_ALLIANCE:

                        if (gamepad1.dpad_right) {

                            if (depositType == DepositType.DT_NORMAL) {

                                if (fieldSide == FieldSide.FS_BLUE) {

                                    bucket.setPosition(bucket_right);

                                } else if (fieldSide == FieldSide.FS_RED) {

                                    bucket.setPosition(bucket_left);
                                }

                                liftHand = LiftHand.LH_RETRACT_ALLIANCE;

                                liftState = LiftState.LS_RELEASED;

                            } else if (depositType == DepositType.DT_SLOW) {

                                bucketTimer.reset();

                                if (fieldSide == FieldSide.FS_BLUE) {

                                    bucket.setPosition(0.6);
                                    liftState = LiftState.LS_RELEASED_SLOW_RIGHT_HALF;

                                } else if (fieldSide == FieldSide.FS_RED) {

                                    bucket.setPosition(0.36);
                                    liftState = LiftState.LS_RELEASED_SLOW_LEFT_HALF;
                                }

                                liftHand = LiftHand.LH_RETRACT_ALLIANCE;

                            }
                        }

                        break;

                    case LS_DEPOSITING_SHARED:

                        if (gamepad1.x) {

                            if (depositType == DepositType.DT_NORMAL) {

                                if (fieldSide == FieldSide.FS_BLUE) {

                                    bucket.setPosition(bucket_left);

                                } else if (fieldSide == FieldSide.FS_RED) {

                                    bucket.setPosition(bucket_right);
                                }

                                liftHand = LiftHand.LH_RETRACT_SHARED;

                                liftState = LiftState.LS_RELEASED;

                            } else if (depositType == DepositType.DT_SLOW) {

                                bucketTimer.reset();

                                if (fieldSide == FieldSide.FS_BLUE) {

                                    bucket.setPosition(0.4);
                                    liftState = LiftState.LS_RELEASED_SLOW_LEFT_HALF;

                                } else if (fieldSide == FieldSide.FS_RED) {

                                    bucket.setPosition(0.56);
                                    liftState = LiftState.LS_RELEASED_SLOW_RIGHT_HALF;

                                }

                                liftHand = LiftHand.LH_RETRACT_SHARED;



                            }
                        }

                        break;


                    case LS_RELEASED_SLOW_RIGHT_HALF:

                        if (bucketTimer.milliseconds() > 500) {

                            bucket.setPosition(bucket_right);

                            liftState = LiftState.LS_RELEASED_SLOW_RIGHT_DUMP;

                        }

                        break;

                    case LS_RELEASED_SLOW_RIGHT_DUMP:

                        if (bucketTimer.milliseconds() > 1000) {

                            bucket.setPosition(bucket_down);

                            liftState = LiftState.LS_RETRACTING;
                        }

                        break;

                    case LS_RELEASED_SLOW_LEFT_HALF:

                        if (bucketTimer.milliseconds() > 500) {

                            bucket.setPosition(bucket_left);

                            liftState = LiftState.LS_RELEASED_SLOW_LEFT_DUMP;

                        }

                        break;

                    case LS_RELEASED_SLOW_LEFT_DUMP:

                        if (bucketTimer.milliseconds() > 1000) {

                            bucket.setPosition(bucket_down);

                            liftState = LiftState.LS_RELEASED;
                        }

                        break;

                    case LS_RELEASED:

                        if (liftHand == LiftHand.LH_RETRACT_ALLIANCE) {

                            if (bucketSensor.alpha() < 70) {

                                bucket.setPosition(bucket_down);

                                // Set lift state to released
                                liftState = LiftState.LS_RETRACTING;

                            }

                        } else if (liftHand == LiftHand.LH_RETRACT_SHARED) {

                            if (bucketSensor.alpha() < 70) {

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

                        if (turretTimer.milliseconds() > 1000) {

                            liftState = LiftState.LS_TURRET_UP;
                        }

                        break;

                    case LS_TURRET_UP:

                        turretTimer.reset();

                        turret.setPosition(turret_center);

                        liftState = LiftState.LS_TURRET_CHECK;

                        break;

                    case LS_TURRET_CHECK:

                        if (turretTimer.milliseconds() > 1000) {

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

                        } else {

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

            } else if (robotState == RobotState.RS_MANUAL) {

                // Manual Lift

                switch (manualLiftState) {

                    case MLS_STATIONARY:

                        if (gamepad2.left_stick_x != 0) {

                            manualLiftState = ManualLiftState.MLS_EXTENDING;
                            strategicDepositTimer.reset();
                        }

                        break;

                    case MLS_EXTENDING:

                        float h = gamepad2.left_stick_x;
                        float b = (float) ((h + 1.0) / 2.0);


                        if (h > 0.0) {

                            int e = (int) (800 * h);

                            if (liftLeft.getCurrentPosition() != e) {

                                liftLeft.setTargetPosition(e);
                                liftRight.setTargetPosition(e);
                                liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                liftLeft.setPower(1.0);
                                liftRight.setPower(1.0);

                                if (strategicDepositTimer.milliseconds() > 250) {

                                    turret.setPosition(b);
                                }
                            }

                        } else if (h == 0.0) {

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

                    arm.setPosition(arm_forward_manual);
                }

                if (gamepad1.a) {

                    arm.setPosition(arm_backward);
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
                                depositHand = DepositHand.DH_STATIONARY;
                                depositState = DepositState.DS_STATIONARY;
                            }

                        } else if (depositHand == DepositHand.DH_RIGHT) {

                            if (bucketTimer.milliseconds() > 500) {

                                bucket.setPosition(bucket_down);
                                depositHand = DepositHand.DH_STATIONARY;
                                depositState = DepositState.DS_STATIONARY;
                            }
                        }

                        break;
                }

                float g = (float) (((gamepad2.left_stick_y) + 1.0) / 2.0);

                if (gamepad2.left_stick_y != 0) {

                    turret.setPosition(g);

                }
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

            /** Reset Encoders **/

            if (gamepad2.left_trigger != 0) {

                liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // set motor ticks to 0
                liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            switch (checkDepositButtonState) {

                case CDBS_STATIONARY:

                    if (gamepad1.dpad_left) {

                        if (depositType == DepositType.DT_NORMAL) {

                            depositType = DepositType.DT_SLOW;

                        } else if (depositType == DepositType.DT_SLOW) {

                            depositType = DepositType.DT_NORMAL;
                        }

                        checkDepositButtonState = CheckDepositButtonState.CDBS_PRESSED;

                    }

                    break;

                case CDBS_PRESSED:

                    if (!gamepad1.dpad_up) {

                        checkDepositButtonState = CheckDepositButtonState.CDBS_STATIONARY;
                    }

                    break;
            }

            switch (checkRobotButtonState) {

                case CRBS_STATIONARY:

                    if (gamepad1.dpad_up) {

                        if (robotState == RobotState.RS_AUTO) {

                            robotState = RobotState.RS_MANUAL;
                            intakeState = IntakeState.IS_STATIONARY;
                            liftState = LiftState.LS_STATIONARY;

                        } else if (robotState == RobotState.RS_MANUAL) {

                            robotState = RobotState.RS_AUTO;
                            manualLiftState = ManualLiftState.MLS_STATIONARY;
                            depositState = DepositState.DS_STATIONARY;

                        }

                        checkRobotButtonState = CheckRobotButtonState.CRBS_PRESSED;

                    }

                    break;

                case CRBS_PRESSED:

                    if (!gamepad1.dpad_right) {

                        checkRobotButtonState = CheckRobotButtonState.CRBS_STATIONARY;

                    }

                    break;
            }

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
            telemetry.addData("Turret: ", turret.getPosition());
            telemetry.addData("Deposit Speed: ", checkDepositButtonState);
            telemetry.addData("Bucket Position: ", bucket.getPosition());
            telemetry.addData("TopRight: ", i_topRight.getPosition());
            telemetry.addData("BottomRight: ", i_bottomRight.getPosition());
            telemetry.addData("G2 Left Stick: ", gamepad2.left_stick_x);
            telemetry.update();
        }
    }
}