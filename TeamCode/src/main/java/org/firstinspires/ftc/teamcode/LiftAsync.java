package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class LiftAsync
{
    DcMotorEx motor1;
    DcMotorEx motor2;
    int targetPos;

    public LiftAsync(DcMotorEx motor1, DcMotorEx motor2, int target)
    {
        this.motor1=motor1;
        this.motor2=motor2;

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        targetPos=target;
    }

    public void setPosition(int target)
    {
        targetPos=target;
    }

    public void adjustLift()
    {
        //write code to PID to the lift here I guess
    }
    public boolean isBusy()
    {
        if(Math.abs(motor1.getCurrentPosition()-targetPos)<5)
        {
            return false;
        }
        return true;
    }
}
