package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class IntakeTest extends LinearOpMode
{
    ServoControl servoControl;
    @Override
    public void runOpMode() throws InterruptedException
    {
        servoControl=new ServoControl(hardwareMap, ServoControl.Side.LEFT);
        servoControl.raiseAllIntakes();

        waitForStart();
        while(opModeIsActive())
        {
            if(gamepad1.left_bumper)
            {
                servoControl.raiseAllIntakes();
            }
            else if(gamepad1.right_bumper)
            {
                servoControl.lowerAllIntakes();
            }
        }
    }
}
