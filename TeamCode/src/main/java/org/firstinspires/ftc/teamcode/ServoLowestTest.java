package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous


public class ServoLowestTest extends LinearOpMode
{
    ServoControl servoControl;
    SensorController sensorControl;
    @Override
    public void runOpMode() throws InterruptedException
    {
        servoControl=new ServoControl(hardwareMap, ServoControl.Side.RIGHT);
        sensorControl=new SensorController(hardwareMap, SensorController.Side.RIGHT);
        servoControl.flipLowest();
        servoControl.openTurretLow();
        waitForStart();
        servoControl.openDepositLow();
        while(!sensorControl.depositNoCube())
        {
            //stall
        }
        //end
    }
}
