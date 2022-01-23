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
public class teleop extends LinearOpMode {

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

        // Reset encoders
        liftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // set motor ticks to 0
        liftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        /*
        0 = not moving
        1 = flip down + intaking
        2 = detected cube + flip up + open flap
        3 = transfering
        4 = transfered + flap closed
        */
        int liftState = 0;
        int liftPosition = 0;
        int liftError = 15;
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
            d_open.setPosition(d_open_minRangeSemi);
            d_bendLeft.setPosition(d_minRange_bendLeft);
            d_bendRight.setPosition(d_minRange_bendRight);

            // Show the elapsed game time & wheel power


            /** Combined Functions **/

            // Resting position - up
            // With a button, intake should flip down, start intake, check if object present, lift up, ex-take into deposit
            // During this time, one flap of deposit should be up and one should be down


            if(leftIntakeState == 0) {
                if(gamepad1.left_bumper) {
                    i_topLeft.setPosition(i_minRange_topLeft);
                    i_bottomLeft.setPosition(i_minRange_bottomLeft);

                    leftIntake.setPower(highSweepPower);

                    // d_open.setPosition(d_open_minRangeSemi);

                    leftIntakeTime.reset();

                    leftIntakeState++;
                }
            }else if((rightIntakeState > 0 || (gamepad1.left_bumper && leftIntakeTime.milliseconds() > 200)) && (leftIntakeState < 9)) {
                leftIntakeState=10;
            }else if (leftIntakeState == 1) {
                if(colorSensor_left.alpha() > 500) {
                    leftIntake.setPower(0.1);

                    i_topLeft.setPosition(i_maxRange_topLeft);
                    i_bottomLeft.setPosition(i_maxRange_bottomLeft);

                    leftIntakeState++;
                }
            }else if(rightIntakeState == 2) {
                leftIntake.setPower(0);

                d_coverLeft.setPosition(d_maxRange_coverLeft);
                d_coverRight.setPosition(d_minRange_coverRight);

                leftIntakeTime.reset();

                leftIntakeState++;
            }else if(leftIntakeState == 3) {
                if(leftIntakeTime.milliseconds() > 1000) {
                    leftIntake.setPower(-highSweepPower);

                    leftIntakeTime.reset();

                    leftIntakeState++;
                }
            }else if(leftIntakeState == 4) {
                if(leftIntakeTime.milliseconds() > 800) {
                    leftIntake.setPower(0);

                    d_open.setPosition(d_open_minRange);

                    d_coverLeft.setPosition(d_minRange_coverLeft);

                    leftIntakeState = 0;
                    objectCaptured = true;
                }
            }else if(leftIntakeState == 10){
                leftIntake.setPower(-1);
                leftIntakeTime.reset();
                leftIntakeState++;
            }else if(leftIntakeState == 11) {
                if(leftIntakeTime.milliseconds() > 800) {
                    leftIntake.setPower(0);

                    i_topLeft.setPosition(i_maxRange_topLeft);
                    i_bottomLeft.setPosition(i_maxRange_bottomLeft);

                    d_open.setPosition(d_open_minRange);

                    d_coverLeft.setPosition(d_minRange_coverLeft);

                    leftIntakeState = 0;
                }
            }else {
                leftIntake.setPower(0);

                i_topLeft.setPosition(i_maxRange_topLeft);
                i_bottomLeft.setPosition(i_maxRange_bottomLeft);

                d_open.setPosition(d_open_minRange);

                d_coverLeft.setPosition(d_minRange_coverLeft);

                leftIntakeState = 0;
            }

            if(rightIntakeState == 0) {
                if(gamepad1.right_bumper) {
                    i_topRight.setPosition(i_minRange_topRight);
                    i_bottomRight.setPosition(i_minRange_bottomRight);

                    rightIntake.setPower(highSweepPower);

                    // d_open.setPosition(d_open_minRangeSemi);

                    rightIntakeTime.reset();

                    rightIntakeState++;
                }
            }else if(leftIntakeState > 0 || (gamepad1.right_bumper && rightIntakeTime.milliseconds() > 200) && (rightIntakeState < 9)) {
                rightIntakeState=10;
            }else if (rightIntakeState == 1) {
                if(colorSensor_right.alpha() > 500) {
                    rightIntake.setPower(0.1);

                    i_topRight.setPosition(i_maxRange_topRight);
                    i_bottomRight.setPosition(i_maxRange_bottomRight);

                    rightIntakeState++;
                }
            }else if(rightIntakeState == 2) {
                rightIntake.setPower(0);

                d_coverRight.setPosition(d_maxRange_coverRight);
                d_coverRight.setPosition(d_minRange_coverRight);

                rightIntakeTime.reset();

                rightIntakeState++;
            }else if(rightIntakeState == 3) {
                if(rightIntakeTime.milliseconds() > 1000) {
                    rightIntake.setPower(-highSweepPower);

                    rightIntakeTime.reset();

                    rightIntakeState++;
                }
            }else if(rightIntakeState == 4) {
                if(rightIntakeTime.milliseconds() > 800) {
                    rightIntake.setPower(0);

                    d_open.setPosition(d_open_minRange);

                    d_coverRight.setPosition(d_minRange_coverRight);

                    rightIntakeState = 0;
                    objectCaptured = true;
                }
            }else if(rightIntakeState == 10){
                rightIntake.setPower(-1);
                rightIntakeTime.reset();
                rightIntakeState++;
            }else if(rightIntakeState == 11) {
                if(rightIntakeTime.milliseconds() > 800) {
                    rightIntake.setPower(0);

                    i_topRight.setPosition(i_maxRange_topRight);
                    i_bottomRight.setPosition(i_maxRange_bottomRight);

                    d_open.setPosition(d_open_minRange);

                    d_coverRight.setPosition(d_minRange_coverRight);

                    rightIntakeState = 0;
                }
            }else {
                rightIntake.setPower(0);

                i_topRight.setPosition(i_maxRange_topRight);
                i_bottomRight.setPosition(i_maxRange_bottomRight);

                d_open.setPosition(d_open_minRange);

                d_coverRight.setPosition(d_minRange_coverRight);

                rightIntakeState = 0;

            }

            /** Lift **/

            // Motor tick count is equal to 384.5
            if(liftState == 0) {
                if(gamepad2.a) {
                    liftFront.setTargetPosition(-shared_targetClose);
                    liftBack.setTargetPosition(-shared_targetClose);
                    liftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftFront.setPower(-1.0);
                    liftBack.setPower(-1.0);
                    liftState = 12;
                    liftPosition = 1;
                }else if(gamepad2.b) {
                    liftFront.setTargetPosition(-shared_targetMiddle);
                    liftBack.setTargetPosition(-shared_targetMiddle);
                    liftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftFront.setPower(-1.0);
                    liftBack.setPower(-1.0);
                    liftState = 13;
                    liftPosition = 1;
                }else if(gamepad2.y) {
                    liftFront.setTargetPosition(-shared_targetFar);
                    liftBack.setTargetPosition(-shared_targetFar);
                    liftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftFront.setPower(-1.0);
                    liftBack.setPower(-1.0);
                    liftState = 14;
                    liftPosition = 1;
                }else if(gamepad2.dpad_down) {
                    liftFront.setTargetPosition(-alliance_middle);
                    liftBack.setTargetPosition(-alliance_middle);
                    liftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftFront.setPower(-1.0);
                    liftBack.setPower(-1.0);
                    liftState = 15;
                    liftPosition = 1;
                }else if(gamepad2.dpad_left) {
                    liftFront.setTargetPosition(-alliance_targetBalanced);
                    liftBack.setTargetPosition(-alliance_targetBalanced);
                    liftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftFront.setPower(-1.0);
                    liftBack.setPower(-1.0);
                    liftState = 16;
                    liftPosition = 1;
                }else if(gamepad2.dpad_up) {
                    liftFront.setTargetPosition(-alliance_targetTipped);
                    liftBack.setTargetPosition(-alliance_targetTipped);
                    liftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftFront.setPower(-1.0);
                    liftBack.setPower(-1.0);
                    liftState = 17;
                    liftPosition = 1;
                }
            }else if(liftState >=10 && liftState <20) {
                if(liftState == 12) {
                    if(Math.abs(liftFront.getCurrentPosition() - shared_targetClose) < liftError) {
                        liftPosition = 12;
                        if(gamepad2.dpad_right || gamepad2.x) {
                            liftState = 20;
                        }
                    }
                }else if(liftState == 13) {
                    if(Math.abs(liftFront.getCurrentPosition() - shared_targetMiddle) < liftError) {
                        liftPosition = 13;
                        if(gamepad2.dpad_right || gamepad2.x) {
                            liftState = 20;
                        }
                    }
                }else if(liftState == 14) {
                    if(Math.abs(liftFront.getCurrentPosition() - shared_targetFar) < liftError) {
                        liftPosition = 14;
                        if(gamepad2.dpad_right || gamepad2.x) {
                            liftState = 20;
                        }
                    }
                }else if(liftState == 15) {
                    if(Math.abs(liftFront.getCurrentPosition() - alliance_middle) < liftError) {
                        liftPosition = 15;
                        if(gamepad2.dpad_right || gamepad2.x) {
                            liftState = 20;
                        }
                    }
                }else if(liftState == 16) {
                    if(Math.abs(liftFront.getCurrentPosition() - alliance_targetBalanced) < liftError) {
                        liftPosition = 16;
                        if(gamepad2.dpad_right || gamepad2.x) {
                            liftState = 20;
                        }
                    }
                }else if(liftState == 17) {
                    if(Math.abs(liftFront.getCurrentPosition() - alliance_targetTipped) < liftError) {
                        liftPosition = 17;
                        if(gamepad2.dpad_right || gamepad2.x) {
                            liftState = 20;
                        }
                    }
                }

                if(liftPosition > 10) {
                    if(gamepad2.a) {
                        liftFront.setTargetPosition(-shared_targetClose);
                        liftBack.setTargetPosition(-shared_targetClose);
                        liftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        if(liftPosition > 12) {
                            liftFront.setPower(1.0);
                            liftBack.setPower(1.0);
                        }else {
                            liftFront.setPower(-1.0);
                            liftBack.setPower(-1.0);
                        }
                        liftState = 12;
                        liftPosition = 1;
                    }else if(gamepad2.b) {
                        liftFront.setTargetPosition(-shared_targetMiddle);
                        liftBack.setTargetPosition(-shared_targetMiddle);
                        liftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftFront.setPower(-1.0);
                        liftBack.setPower(-1.0);
                        if(liftPosition > 13) {
                            liftFront.setPower(1.0);
                            liftBack.setPower(1.0);
                        }else {
                            liftFront.setPower(-1.0);
                            liftBack.setPower(-1.0);
                        }
                        liftState = 13;
                        liftPosition = 1;
                    }else if(gamepad2.y) {
                        liftFront.setTargetPosition(-shared_targetFar);
                        liftBack.setTargetPosition(-shared_targetFar);
                        liftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftFront.setPower(-1.0);
                        liftBack.setPower(-1.0);
                        if(liftPosition > 14) {
                            liftFront.setPower(1.0);
                            liftBack.setPower(1.0);
                        }else {
                            liftFront.setPower(-1.0);
                            liftBack.setPower(-1.0);
                        }
                        liftState = 14;
                        liftPosition = 1;
                    }else if(gamepad2.dpad_down) {
                        liftFront.setTargetPosition(-alliance_middle);
                        liftBack.setTargetPosition(-alliance_middle);
                        liftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftFront.setPower(-1.0);
                        liftBack.setPower(-1.0);
                        if(liftPosition > 15) {
                            liftFront.setPower(1.0);
                            liftBack.setPower(1.0);
                        }else {
                            liftFront.setPower(-1.0);
                            liftBack.setPower(-1.0);
                        }
                        liftState = 15;
                        liftPosition = 1;
                    }else if(gamepad2.dpad_left) {
                        liftFront.setTargetPosition(-alliance_targetBalanced);
                        liftBack.setTargetPosition(-alliance_targetBalanced);
                        liftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftFront.setPower(-1.0);
                        liftBack.setPower(-1.0);
                        if(liftPosition > 16) {
                            liftFront.setPower(1.0);
                            liftBack.setPower(1.0);
                        }else {
                            liftFront.setPower(-1.0);
                            liftBack.setPower(-1.0);
                        }
                        liftState = 16;
                        liftPosition = 1;
                    }else if(gamepad2.dpad_up) {
                        liftFront.setTargetPosition(-alliance_targetTipped);
                        liftBack.setTargetPosition(-alliance_targetTipped);
                        liftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftFront.setPower(-1.0);
                        liftBack.setPower(-1.0);
                        if(liftPosition > 17) {
                            liftFront.setPower(1.0);
                            liftBack.setPower(1.0);
                        }else {
                            liftFront.setPower(-1.0);
                            liftBack.setPower(-1.0);
                        }
                        liftState = 17;
                        liftPosition = 1;
                    }
                }
            }else if(liftState == 20) {
                //moving the deposit to correct angle and depositing
                //fill in these values idk if these are right
                if(liftPosition == 12) {
                    d_bendLeft.setPosition(d_maxRange_bendLeft);
                    d_bendRight.setPosition(d_maxRange_bendRight);
                    d_open.setPosition(d_open_shared);
                }else if(liftPosition == 13) {
                    d_bendLeft.setPosition(d_maxRange_bendLeft);
                    d_bendRight.setPosition(d_maxRange_bendRight);
                    d_open.setPosition(d_open_shared);
                }else if(liftPosition == 14) {
                    d_bendLeft.setPosition(d_maxRange_bendLeft);
                    d_bendRight.setPosition(d_maxRange_bendRight);
                    d_open.setPosition(d_open_shared);
                }else if(liftPosition == 15) {
                    d_bendLeft.setPosition(d_maxRange_bendLeft);
                    d_bendRight.setPosition(d_maxRange_bendRight);
                    d_open.setPosition(d_open_shared);
                }else if(liftPosition == 16) {
                    d_bendLeft.setPosition(d_minRange_bendLeft);
                    d_bendRight.setPosition(d_minRange_bendRight);
                    d_open.setPosition(d_open_top);
                }else if(liftPosition == 17) {
                    d_bendLeft.setPosition(d_minRange_bendLeft);
                    d_bendRight.setPosition(d_minRange_bendRight);
                    d_open.setPosition(d_open_top);
                }
                liftTime.reset();
                liftState = 21;
            }else if(liftState == 21) {
                if(liftTime.milliseconds() > 600) {
                    d_open.setPosition(d_open_minRange);
                    d_bendLeft.setPosition(d_minRange_bendLeft);
                    d_bendRight.setPosition(d_minRange_bendRight);

                    liftTime.reset();
                    liftState = 22;
                }
            }else if(liftState == 22) {
                if(liftTime.milliseconds() > 500 || liftPosition > 15 /* this is risky idk if we keep it*/) {
                    liftFront.setTargetPosition(0);
                    liftBack.setTargetPosition(0);
                    liftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftFront.setPower(1.0);
                    liftBack.setPower(1.0);
                    liftState = 23;
                }
            }else if(liftState == 23) {
                if(Math.abs(liftFront.getCurrentPosition() - 0) < liftError) {
                    liftTime.reset();
                    liftState = 24;
                }
            }else if(liftState == 24) {
                if(liftTime.milliseconds() > 150) {
                    liftFront.setPower(0);
                    liftBack.setPower(0);
                    liftPosition = 0;
                    liftState = 0;
                }
            }

            // drivetrain

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

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "leftFront (%.2f), leftBack (%.2f), rightFront (%.2f), rightBack (%.2f)",
                    leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
            telemetry.addData("Color - Left", colorSensor_left.alpha());
            telemetry.addData("Color - Right", colorSensor_right.alpha());
            telemetry.addData("Lift - Front:", liftFront.getCurrentPosition());
            telemetry.addData("leftIntakeState:", leftIntakeState);
            telemetry.addData("rightIntakeState:", rightIntakeState);
            telemetry.addData("liftState:", liftState);
            telemetry.addData("listPosition:", liftPosition);
            telemetry.update();

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

            /** Reset Encoders **/

            if (gamepad2.right_trigger != 0) {
                liftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // set motor ticks to 0
            }liftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}
