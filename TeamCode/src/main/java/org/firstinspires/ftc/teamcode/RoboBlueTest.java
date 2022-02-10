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
    ColorRangeSensor colorSensor_left;
    ColorRangeSensor colorSensor_right;
    Rev2mDistanceSensor depositSensor;

    // Runtime
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime depositTimer = new ElapsedTime();

    LiftState liftState = LiftState.LIFT_EXTENDING;

    enum LiftState {
        LIFT_EXTENDING,
        LIFT_EXTENDING_BALANCED,
        LIFT_EXTENDING_TIPPED,
        LIFT_EXTENDING_NEAR,
        LIFT_EXTENDING_MIDDLE,
        LIFT_EXTENDING_FAR,
        LIFT_TARGET,
        LIFT_TARGET_BALANCED,
        LIFT_TARGET_TIPPED,
        LIFT_TARGET_NEAR,
        LIFT_TARGET_MIDDLE,
        LIFT_TARGET_FAR,
        LIFT_DEPOSITING,
        LIFT_RELEASED,
        LIFT_RETRACTING
    }

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
        depositSensor = hardwareMap.get(Rev2mDistanceSensor.class, "depositSensor");

        // Intake
        double highSweepPower = 0.8;
        // double lowSweepPower = 0.4;

        // minRange - intake down, deposit position closed, deposit folded down
        // maxRange - intake up, deposit position open, deposit in scoring position

        // Deposit servo positions
        double d_open_minRange = 0.65;
        double d_open_minRangeSemi = 0.63;
        double d_open_top = 0.53;
        double d_open_middle = 0.49;   // Fix this value
        double d_open_shared = 0.47;
        double d_minRange_coverLeft = 0.59;
        double d_maxRange_coverLeft = 0.15;
        double d_minRange_coverRight = 0.41;
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
        int TARGET_TIPPED = 625;
        int TARGET_BALANCED = 575;   // Fix this value
        int TARGET_CENTER = 550;   // Fix this value
        int TARGET_NEAR = 120;
        int TARGET_MIDDLE = 200;
        int TARGET_FAR = 280;

        // Reset encoders
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // set motor ticks to 0
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        d_open.setPosition(d_open_minRangeSemi);
        d_bendLeft.setPosition(d_minRange_bendLeft);
        d_bendRight.setPosition(d_minRange_bendRight);

        d_coverLeft.setPosition(d_minRange_coverLeft);
        d_coverRight.setPosition(d_minRange_coverRight);

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

                    d_open.setPosition(d_open_minRangeSemi);

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

                d_coverLeft.setPosition(d_maxRange_coverLeft);
                d_coverRight.setPosition(d_minRange_coverRight);

                leftIntakeTime.reset();

                leftIntakeState++;
            } else if (leftIntakeState == 3) {
                if (leftIntakeTime.milliseconds() > intakeFlipUpTime) {
                    leftIntake.setPower(-highSweepPower);

                    leftIntakeTime.reset();

                    leftIntakeState++;
                }
            } else if (leftIntakeState == 4) {
                if (/*colorSensor_left.getDistance(DistanceUnit.CM) > intakeEjectDistance*/ depositSensor.getDistance(DistanceUnit.CM) < 9.0) {
                    leftIntake.setPower(0);

                    d_open.setPosition(d_open_minRangeSemi);

                    d_coverLeft.setPosition(d_minRange_coverLeft);

                    leftIntakeState = 0;
                    objectCaptured = true;
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

                d_open.setPosition(d_open_minRangeSemi);

                d_coverLeft.setPosition(d_minRange_coverLeft);

                leftIntakeState = 0;
                // }
            } else {
                leftIntake.setPower(0);

                i_topLeft.setPosition(i_maxRange_topLeft);
                i_bottomLeft.setPosition(i_maxRange_bottomLeft);

                // d_open.setPosition(d_open_minRangeSemi);

                d_coverLeft.setPosition(d_minRange_coverLeft);

                leftIntakeState = 0;
            }

            if (rightIntakeState == 0) {
                if (gamepad1.right_bumper) {

                    i_topRight.setPosition(i_minRange_topRight);
                    i_bottomRight.setPosition(i_minRange_bottomRight);

                    rightIntake.setPower(highSweepPower);

                    d_open.setPosition(d_open_minRangeSemi);

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

                d_coverRight.setPosition(d_maxRange_coverRight);
                d_coverLeft.setPosition(d_minRange_coverLeft);

                rightIntakeTime.reset();

                rightIntakeState++;
            } else if (rightIntakeState == 3) {
                if (rightIntakeTime.milliseconds() > intakeFlipUpTime) {
                    rightIntake.setPower(-highSweepPower);

                    rightIntakeTime.reset();

                    rightIntakeState++;
                }
            } else if (rightIntakeState == 4) {
                if (/*colorSensor_right.getDistance(DistanceUnit.CM) > intakeEjectDistance*/depositSensor.getDistance(DistanceUnit.CM) < 9.0) {
                    rightIntake.setPower(0);

                    d_open.setPosition(d_open_minRangeSemi);

                    d_coverRight.setPosition(d_minRange_coverRight);

                    rightIntakeState = 0;
                    objectCaptured = true;
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

                d_open.setPosition(d_open_minRangeSemi);

                d_coverRight.setPosition(d_minRange_coverRight);

                rightIntakeState = 0;
                // }
            } else {
                rightIntake.setPower(0);

                i_topRight.setPosition(i_maxRange_topRight);
                i_bottomRight.setPosition(i_maxRange_bottomRight);

                // d_open.setPosition(d_open_minRangeSemi);

                d_coverRight.setPosition(d_minRange_coverRight);

                rightIntakeState = 0;

            }

            /** Lift **/

            // Motor tick count is equal to 384.5

            switch (liftState) {
                case LIFT_EXTENDING:

                    // Check if button was pressed
                    if (gamepad2.dpad_up) {

                        // Run lift
                        liftLeft.setTargetPosition(-TARGET_TIPPED);
                        liftRight.setTargetPosition(-TARGET_TIPPED);
                        liftLeft.setPower(-1.0);
                        liftRight.setPower(-1.0);

                        // "10" is arbitrary - might need to be adjusted
                        if ((Math.abs(liftLeft.getCurrentPosition() - TARGET_TIPPED)) > 10) {

                            // Run lift
                            liftLeft.setTargetPosition(-TARGET_TIPPED);
                            liftRight.setTargetPosition(-TARGET_TIPPED);
                            liftLeft.setPower(-1.0);
                            liftRight.setPower(-1.0);

                            // Ensure movement of drivetrain during while loop
                            y = -gamepad1.right_stick_x; // Reversed
                            x = gamepad1.left_stick_x * 1.1; // Strafing + Precision
                            rx = -gamepad1.left_stick_y; // Forward/Backward

                            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                            leftFrontPower = (y + x - rx) / denominator;
                            leftBackPower = (y - x - rx) / denominator;
                            rightFrontPower = (y - x + rx) / denominator;
                            rightBackPower = (y + x + rx) / denominator;

                            leftFront.setPower(leftBackPower);
                            leftBack.setPower(leftFrontPower);
                            rightFront.setPower(rightFrontPower);
                            rightBack.setPower(rightBackPower);

                            // Set lift state to target
                            liftState = liftState.LIFT_TARGET;

                        } else {

                            liftState = LiftState.LIFT_EXTENDING;
                        }

                        break;

                    } else if (gamepad2.dpad_left) {

                        // Run lift
                        liftLeft.setTargetPosition(-TARGET_TIPPED);
                        liftRight.setTargetPosition(-TARGET_TIPPED);
                        liftLeft.setPower(-1.0);
                        liftRight.setPower(-1.0);

                        // "10" is arbitrary - might need to be adjusted
                        if ((Math.abs(liftLeft.getCurrentPosition() - TARGET_BALANCED)) > 10) {

                            // Run lift
                            liftLeft.setPower(-1.0);
                            liftRight.setPower(-1.0);
                            liftLeft.setTargetPosition(-TARGET_BALANCED);
                            liftRight.setTargetPosition(-TARGET_BALANCED);

                            // Ensure movement of drivetrain during while loop
                            double y = -gamepad1.right_stick_x; // Reversed
                            double x = gamepad1.left_stick_x * 1.1; // Strafing + Precision
                            double rx = -gamepad1.left_stick_y; // Forward/Backward

                            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                            double leftFrontPower = (y + x - rx) / denominator;
                            double leftBackPower = (y - x - rx) / denominator;
                            rightFrontPower = (y - x + rx) / denominator;
                            rightBackPower = (y + x + rx) / denominator;

                            leftFront.setPower(leftBackPower);
                            leftBack.setPower(leftFrontPower);
                            rightFront.setPower(rightFrontPower);
                            rightBack.setPower(rightBackPower);

                            // Set lift state to target
                            liftState = LiftState.LIFT_TARGET;

                        } else {

                            liftState = LiftState.LIFT_EXTENDING;
                        }

                        break;

                    } else if (gamepad2.y) {

                        // Run lift
                        liftLeft.setTargetPosition(-TARGET_TIPPED);
                        liftRight.setTargetPosition(-TARGET_TIPPED);
                        liftLeft.setPower(-1.0);
                        liftRight.setPower(-1.0);

                        // "10" is arbitrary - might need to be adjusted
                        if ((Math.abs(liftLeft.getCurrentPosition() - TARGET_FAR) > 5)) {

                            // Run lift
                            liftLeft.setTargetPosition(-TARGET_FAR);
                            liftRight.setTargetPosition(-TARGET_FAR);
                            liftLeft.setPower(-1.0);
                            liftRight.setPower(-1.0);

                            // Ensure movement of drivetrain during while loop
                            y = -gamepad1.right_stick_x; // Reversed
                            x = gamepad1.left_stick_x * 1.1; // Strafing + Precision
                            rx = -gamepad1.left_stick_y; // Forward/Backward

                            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                            leftFrontPower = (y + x - rx) / denominator;
                            leftBackPower = (y - x - rx) / denominator;
                            rightFrontPower = (y - x + rx) / denominator;
                            rightBackPower = (y + x + rx) / denominator;

                            leftFront.setPower(leftBackPower);
                            leftBack.setPower(leftFrontPower);
                            rightFront.setPower(rightFrontPower);
                            rightBack.setPower(rightBackPower);

                            // Set lift state to target
                            liftState = LiftState.LIFT_TARGET;

                        } else {

                            liftState = LiftState.LIFT_EXTENDING;
                        }

                        break;

                    } else if (gamepad2.b) {

                        // Run lift
                        liftLeft.setTargetPosition(-TARGET_TIPPED);
                        liftRight.setTargetPosition(-TARGET_TIPPED);
                        liftLeft.setPower(-1.0);
                        liftRight.setPower(-1.0);

                        // "10" is arbitrary - might need to be adjusted
                        if ((Math.abs(liftLeft.getCurrentPosition() - TARGET_MIDDLE)) > 5) {

                            // Run lift
                            liftLeft.setTargetPosition(-TARGET_MIDDLE);
                            liftRight.setTargetPosition(-TARGET_MIDDLE);
                            liftLeft.setPower(-1.0);
                            liftRight.setPower(-1.0);

                            // Ensure movement of drivetrain during while loop
                            y = -gamepad1.right_stick_x; // Reversed
                            x = gamepad1.left_stick_x * 1.1; // Strafing + Precision
                            rx = -gamepad1.left_stick_y; // Forward/Backward

                            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                            leftFrontPower = (y + x - rx) / denominator;
                            leftBackPower = (y - x - rx) / denominator;
                            rightFrontPower = (y - x + rx) / denominator;
                            rightBackPower = (y + x + rx) / denominator;

                            leftFront.setPower(leftBackPower);
                            leftBack.setPower(leftFrontPower);
                            rightFront.setPower(rightFrontPower);
                            rightBack.setPower(rightBackPower);

                            // Set lift state to target
                            liftState = LiftState.LIFT_TARGET;

                        } else {

                            liftState = LiftState.LIFT_EXTENDING;

                        }

                        break;

                    } else if (gamepad2.a) {

                        // Run lift
                        liftLeft.setTargetPosition(-TARGET_TIPPED);
                        liftRight.setTargetPosition(-TARGET_TIPPED);
                        liftLeft.setPower(-1.0);
                        liftRight.setPower(-1.0);

                        // "10" is arbitrary - might need to be adjusted
                        if ((Math.abs(liftLeft.getCurrentPosition() - TARGET_NEAR) > 5)) {

                            // Run lift
                            liftLeft.setTargetPosition(-TARGET_NEAR);
                            liftRight.setTargetPosition(-TARGET_NEAR);
                            liftFront.setPower(-1.0);
                            liftBack.setPower(-1.0);

                            // Ensure movement of drivetrain during while loop
                            y = -gamepad1.right_stick_x; // Reversed
                            x = gamepad1.left_stick_x * 1.1; // Strafing + Precision
                            rx = -gamepad1.left_stick_y; // Forward/Backward

                            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                            leftFrontPower = (y + x - rx) / denominator;
                            leftBackPower = (y - x - rx) / denominator;
                            rightFrontPower = (y - x + rx) / denominator;
                            rightBackPower = (y + x + rx) / denominator;

                            leftFront.setPower(leftBackPower);
                            leftBack.setPower(leftFrontPower);
                            rightFront.setPower(rightFrontPower);
                            rightBack.setPower(rightBackPower);

                            // Set lift state to target
                            liftState = RoboBoss.LiftState.LIFT_TARGET;

                        } else {

                            liftState = RoboBoss.LiftState.LIFT_EXTENDING;
                        }

                        break;
                    }

                case LIFT_TARGET:

                    // Check if the lift has fully extended
                    if (gamepad2.dpad_up && (Math.abs(liftFront.getCurrentPosition() - TARGET_TIPPED)) < 10) {

                        // Set state to depositing
                        liftState = RoboBoss.LiftState.LIFT_DEPOSITING;
                        break;

                    } else if (gamepad2.dpad_left && (Math.abs(liftFront.getCurrentPosition() - TARGET_BALANCED)) < 10) {

                        // Set state to depositing
                        liftState = RoboBoss.LiftState.LIFT_DEPOSITING;
                        break;

                    } else if (gamepad2.y && (Math.abs(liftFront.getCurrentPosition() - TARGET_FAR)) < 5) {

                        // Set state to depositing
                        liftState = RoboBoss.LiftState.LIFT_DEPOSITING;
                        break;

                    } else if (gamepad2.b && (Math.abs(liftFront.getCurrentPosition() - TARGET_MIDDLE)) < 5) {

                        // Set state to depositing
                        liftState = RoboBoss.LiftState.LIFT_DEPOSITING;
                        break;

                    } else if (gamepad2.a && (Math.abs(liftFront.getCurrentPosition() - TARGET_NEAR)) < 5) {

                        // Set state to depositing
                        liftState = RoboBoss.LiftState.LIFT_DEPOSITING;
                        break;
                    }

                case LIFT_DEPOSITING:
                    if (gamepad2.dpad_right || gamepad2.x) {

                        d_open.setPosition(d_open_top);

                        if (depositTimer.milliseconds() <= 500) {

                            d_open.setPosition(d_open_top);

                            // Keep lift in place while depositing
                            liftFront.setTargetPosition(0);
                            liftBack.setTargetPosition(0);
                            liftFront.setPower(-1.0);
                            liftBack.setPower(-1.0);

                            // Ensure movement of drivetrain during while loop
                            y = -gamepad1.right_stick_x; // Reversed
                            x = gamepad1.left_stick_x * 1.1; // Strafing + Precision
                            rx = -gamepad1.left_stick_y; // Forward/Backward

                            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                            leftFrontPower = (y + x - rx) / denominator;
                            leftBackPower = (y - x - rx) / denominator;
                            rightFrontPower = (y - x + rx) / denominator;
                            rightBackPower = (y + x + rx) / denominator;

                            leftFront.setPower(leftBackPower);
                            leftBack.setPower(leftFrontPower);
                            rightFront.setPower(rightFrontPower);
                            rightBack.setPower(rightBackPower);

                        }

                        d_open.setPosition(d_open_minRange);

                        depositTimer.reset();
                        // Set lift state to released
                        liftState = RoboBoss.LiftState.LIFT_RELEASED;

                        break;
                    }

                case LIFT_RELEASED:
                    if (d_open.getPosition() == d_open_minRange) {

                        // Retract lift
                        liftFront.setTargetPosition(0);
                        liftFront.setTargetPosition(0);
                        liftFront.setPower(1.0);
                        liftFront.setPower(1.0);

                        if ((Math.abs(liftFront.getCurrentPosition() - LIFT_IDLE)) != 0) {

                            // Retract lift
                            liftFront.setTargetPosition(0);
                            liftFront.setTargetPosition(0);
                            liftFront.setPower(1.0);
                            liftFront.setPower(1.0);

                            // Ensure movement of drivetrain during while loop
                            y = -gamepad1.right_stick_x; // Reversed
                            x = gamepad1.left_stick_x * 1.1; // Strafing + Precision
                            rx = -gamepad1.left_stick_y; // Forward/Backward

                            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                            leftFrontPower = (y + x - rx) / denominator;
                            leftBackPower = (y - x - rx) / denominator;
                            rightFrontPower = (y - x + rx) / denominator;
                            rightBackPower = (y + x + rx) / denominator;

                            leftFront.setPower(leftBackPower);
                            leftBack.setPower(leftFrontPower);
                            rightFront.setPower(rightFrontPower);
                            rightBack.setPower(rightBackPower);


                            // Set lift state to retracting
                            liftState = RoboBoss.LiftState.LIFT_RETRACTING;

                        } else {

                            liftState = RoboBoss.LiftState.LIFT_RELEASED;
                        }

                        break;
                    }

                case LIFT_RETRACTING:
                    // Check if lift is fully retracted
                    if ((Math.abs(liftFront.getCurrentPosition() - LIFT_IDLE)) == 0) {

                        // Set lift state to empty
                        liftState = RoboBoss.LiftState.LIFT_EXTENDING;

                        break;
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

                carousel.setPower(0.5 + 0.5 * (int) a);

            } else if (gamepad2.right_trigger != 0) {

                carousel.setPower(-(0.5 + 0.5 * (int) a));
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
            telemetry.addData("Left Front Encoders:", leftFront.getCurrentPosition());
            telemetry.addData("Left Back Encoders: ", leftBack.getCurrentPosition());
            telemetry.addData("Right Front Encoders: ", rightFront.getCurrentPosition());
            telemetry.addData("Right Back Encoders: ", rightBack.getCurrentPosition());
            telemetry.addData("Color - Left", colorSensor_left.getDistance(DistanceUnit.CM));
            telemetry.addData("Color - Right", colorSensor_right.getDistance(DistanceUnit.CM));
            telemetry.addData("Lift - Front:", liftLeft.getCurrentPosition());
            telemetry.addData("Lift - Back:", liftRight.getCurrentPosition());
            telemetry.addData("Carousel: ", carousel.getPower());
            telemetry.addData("Lift State: ", liftState);
            telemetry.addData("Lift Position: ", liftPosition);
            telemetry.update();

        }
    }
}
