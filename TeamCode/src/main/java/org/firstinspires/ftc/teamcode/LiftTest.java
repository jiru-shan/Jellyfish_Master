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
    ElapsedTime timer;
    DcMotorEx motor1, motor2;
    public static int position=350;
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
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);



        timer=new ElapsedTime();
        lift=new LiftAsync(hardwareMap, 0);
        servoControl=new ServoControl(hardwareMap, ServoControl.Side.LEFT);

        servoControl.closeDeposit();
        servoControl.startingPos();
        //servoControl.setTurret(turretPos);
        servoControl.raiseAllIntakes();
        waitForStart();

        //servoControl.flipOut();
        servoControl.prepDeposit();
        timer.reset();

        servoControl.flipMedium();
        motor1.setTargetPosition(-position);
        motor2.setTargetPosition(position);

        while(Math.abs(motor1.getCurrentPosition())<position)
        {
            if(motor2.getCurrentPosition()>100)
            {
                servoControl.openTurret();
            }
            motor2.setPower(1);
            motor1.setPower(-1);

            packet.put("motor 1:", motor1.getCurrentPosition());
            packet.put("motor 2:", motor2.getCurrentPosition());

            dashboard.sendTelemetryPacket(packet);
            telemetry.addData(">1: ", motor1.getCurrentPosition());
            telemetry.addData(">2: ", motor2.getCurrentPosition());
            telemetry.update();
        }
        servoControl.openDeposit();
        motor1.setPower(0);
        motor2.setPower(0);

        timer.reset();
        while(timer.milliseconds()<500)
        {

        }

        servoControl.startingPos();
        motor1.setTargetPosition(0);
        motor2.setTargetPosition(0);
        while(Math.abs(motor1.getCurrentPosition())>5)
        {
            motor2.setPower(-1);
            motor1.setPower(1);
        }
        motor1.setPower(0);
        motor2.setPower(0);
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
