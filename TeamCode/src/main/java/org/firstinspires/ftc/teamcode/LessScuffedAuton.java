package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.VisionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous

/* TO DO
* Create/test a path
* Connect to bot via ADB Wireless
* Check if FTC Dashboard works
* Test Vision
* Find arm encoder positions
* Do RR some time in the future
* */

//Red Side Auton

public class LessScuffedAuton extends LinearOpMode
{
    private DcMotor Bl ;
    private DcMotor Br;
    private DcMotor Fl;
    private DcMotor Fr;
    private DcMotor Arm;
    private DcMotor Slides;


    //private ElapsedTime runtime = new ElapsedTime();

    private CRServo Sv;
    //CRServo for continuous

    BNO055IMU imu;

    AutonDrive drive;
    Lift lift;
    Deposit deposit;

    OpenCvWebcam camera;
    VisionPipeline pipeline;

    Controller control;
    int barcodePos;

    SingleMotorMechanism temp;


    static final int[] ARM_POSITIONS={0, 100}; //temp values


    @Override
    public void runOpMode() throws InterruptedException
    {
       initialize();
       drive=new AutonDrive(new DcMotor [] {Fl,Bl ,Fr,Br}, imu, telemetry, control);
       lift =new Lift(Slides);
       deposit=new Deposit(Arm);
       Bl.setPower(50);


        waitForStart();
        if(opModeIsActive())
        {
            telemetry.addData(">", "3:23 Edition");    //
            telemetry.update();
           /* barcodePos=pipeline.getAnalysis();
            telemetry.addData("Position", barcodePos);
            telemetry.update();
            camera.stopStreaming();
            camera.closeCameraDevice();*/
            drive.moveInches(2.25, 0.5, 0, 2);
            drive.turnDegrees(87, 0.8, 3);
            drive.moveInches(16.5, 0.5, 0, 3);
            double tempTarget=SystemClock.uptimeMillis()+1000;
            while(SystemClock.uptimeMillis()<tempTarget)
            {

            }
            drive.moveInches(3, 0.1, 0, 1.5);
            double servoTarget=SystemClock.uptimeMillis()+3000;
            while(SystemClock.uptimeMillis()<servoTarget)
            {
                Sv.setPower(-1);
            }
            Sv.setPower(0);
            drive.moveInches(-28, 0.5, 0, 5);
            drive.turnDegrees(89, 0.8, 3);
            drive.moveInches(-10, 0.5, 0, 3);
            //barcodePos=pipeline.getAnalysis();
            //lift.moveToLevel(barcodePos, 0.5);
            deposit.moveToPos(ARM_POSITIONS[1], 0.5);

            double target=SystemClock.uptimeMillis()+1000;
            while(SystemClock.uptimeMillis()<target)
            {
                //do nothing
            }
            deposit.moveToPos(ARM_POSITIONS[0], 0.5);
            drive.turnDegrees(-87, 0.8, 5);
            //drive.moveInches(-70, 1, 0, 15);
            drive.movePipes(1, -1, 8);

        }

    }


    public void initialize()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        Bl=hardwareMap.get(DcMotor.class, "leftBack");
        Br=hardwareMap.get(DcMotor.class, "rightBack");
        Fr=hardwareMap.get(DcMotor.class, "rightFront");
        Fl=hardwareMap.get(DcMotor.class, "leftFront");
        Arm =hardwareMap.get(DcMotor.class, "armRotate");
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slides=hardwareMap.get(DcMotor.class, "armLift");

        temp=new SingleMotorMechanism(Br);

        Sv=hardwareMap.get(CRServo.class, "carousel");

        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        imu.initialize(parameters);

        while(!imu.isGyroCalibrated()){}

        control=new BasicController();
        webcamInit();


        telemetry.addData(">", "3:25 Edition");    //
        telemetry.update();

    }

        public void webcamInit()
        {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
            pipeline=new VisionPipeline(telemetry);
            camera.setPipeline(pipeline);
            camera.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode)
                {
                    telemetry.addData("Status:", "pain");
                    telemetry.update();
                }
            });
        }
}
