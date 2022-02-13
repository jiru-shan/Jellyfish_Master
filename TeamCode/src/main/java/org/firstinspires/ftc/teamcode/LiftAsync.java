package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LiftAsync
{
    DcMotorEx motor1;
    DcMotorEx motor2;

    int targetPos;
    int currentPos;

    int error;
    double derivative;
    double motorPower;
    int previousError=0;
    int integralSum=0;

    //define kP, kI, and kD by actually testing
    double kP=5;
    double kI=0.2;
    double kD=1.1;

    ElapsedTime timer;

    public LiftAsync(HardwareMap hwMap, int target)
    {
        this.motor1=hwMap.get(DcMotorEx.class, "liftLeft");
        this.motor2=hwMap.get(DcMotorEx.class, "liftRight");

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        targetPos=target;
        timer=new ElapsedTime();
    }

    public void setPosition(int target)
    {
        targetPos=target;
    }

    public void adjustLift()
    {
        currentPos=motor1.getCurrentPosition();

        error=targetPos-currentPos;
        integralSum+=error*timer.milliseconds();
        derivative=(error-previousError)/timer.milliseconds();

        motorPower=(error*kP)+(integralSum*kI)+(derivative*kD);
        motor1.setPower(motorPower/100);
        motor2.setPower(motorPower/100);

        previousError=error;
        timer.reset();
    }
    public boolean isBusy()
    {
        if(Math.abs(motor1.getCurrentPosition()-targetPos)<5)
        {
            return false;
        }
        return true;
    }
    public void resetLiftTimer()
    {
        timer.reset();
    }
}
