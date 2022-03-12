package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ServoTest extends LinearOpMode
{
    ServoControl servoControl;
    //public static double deposit;

    @Override
    public void runOpMode() throws InterruptedException
    {
        servoControl=new ServoControl(hardwareMap, ServoControl.Side.LEFT);
        servoControl.prepDeposit();
        //servoControl.openDeposit();
        //servoControl.openDeposit();
        //servoControl.flipMedium();
        waitForStart();
        while(opModeIsActive())
        {
            servoControl.openDepositIntake();
            //bruh
        }
    }
}
