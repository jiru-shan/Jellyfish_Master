package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@TeleOp
public class LiftTest extends LinearOpMode
{
    LiftAsync lift;
    ServoControl servoControl;
    SensorController sensorControl;
    ElapsedTime timer;
    DcMotorEx motor1, motor2;
    public static int position=430;
    //lift medium level 430
    public static double turretPos=0.35;
    FtcDashboard dashboard;
    TelemetryPacket packet;

    @Override
    public void runOpMode() throws InterruptedException
    {
        dashboard=FtcDashboard.getInstance();
        packet=new TelemetryPacket();
        motor1=hardwareMap.get(DcMotorEx.class, "liftLeft");
        motor2=hardwareMap.get(DcMotorEx.class, "liftRight");
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        timer=new ElapsedTime();
        lift=new LiftAsync(hardwareMap, 0);
        servoControl=new ServoControl(hardwareMap, ServoControl.Side.RIGHT);
        sensorControl=new SensorController(hardwareMap, SensorController.Side.RIGHT);

        servoControl.closeDeposit();
        servoControl.startingPos();
        //servoControl.setTurret(turretPos);
        servoControl.raiseAllIntakes();
        waitForStart();

        //servoControl.flipMedium();
        servoControl.prepDeposit();

        timer.reset();

        lift.setPosition(position);
        //servoControl.flipMedium();
        int i=0;
        while(lift.isBusy())
        {
            if(lift.getPos2()>100)
            {
                servoControl.openTurret();
                servoControl.flipMiddle_level();
            }
            i++;
            lift.adjustLift();

            packet.put("motor 1:", lift.getPos1());
            packet.put("motor 2:", lift.getPos2());
            packet.put("motor power: ", lift.getPower());
            packet.put("vel deg: ", lift.getVelocity());
            packet.put("is busy?: ", lift.isBusy());
            packet.put("test: ", i);

            dashboard.sendTelemetryPacket(packet);
            telemetry.addData(">1: ", lift.getPos1());
            telemetry.addData(">2: ", lift.getPos2());
            telemetry.update();
        }
       // servoControl.openTurret();
        packet.put("is busy?: ", lift.isBusy());
        dashboard.sendTelemetryPacket(packet);
        servoControl.openDepositMiddleLevel();

        boolean b=true;
        while(b)
        {
            if(sensorControl.depositNoCube())
            {
                b=false;
            }
        }
        servoControl.flipOut();
        servoControl.startingPos();
        packet.put("exited: ", "true");
        dashboard.sendTelemetryPacket(packet);
        lift.setPosition(0, 0.8);
        while(lift.isBusy())
        {
            if(lift.getPos2()<50)
            {
                servoControl.returnArm();
            }
            lift.adjustLift();
            packet.put("motor 1:", lift.getPos1());
            packet.put("motor 2:", lift.getPos2());
            packet.put("motor power: ", lift.getPower());
        }
       // while(motor1.getCurrentPosition()<330)
       // {
         //   motor1.setPower(1);
          //  motor2.setPower(1);
       // }
        //lift.brake();
       /* lift.setPosition(position);
       // timer.reset();
        while(lift.isBusy())
        {
            packet.put("pow", lift.motorPower);

            dashboard.sendTelemetryPacket(packet);

            telemetry.addData(">", lift.getPos1());
            telemetry.addData(">2", lift.getPos2());
            telemetry.update();
            lift.adjustLift();
        }
        servoControl.openDeposit();
        double target= SystemClock.uptimeMillis()+500;
        while(SystemClock.uptimeMillis()<target)
        {
            packet.put("motor 1", lift.getPos1());
            packet.put("motor 2", lift.getPos2());
            dashboard.sendTelemetryPacket(packet);
            lift.setPower(0);
            //lift.adjustLift();
        }
        servoControl.startingPos();
        lift.setPosition(0);
        while(lift.isBusy())
        {
            lift.adjustLift();
        }*/
    }
}
