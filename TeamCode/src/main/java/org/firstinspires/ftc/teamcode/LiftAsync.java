package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    int previousPosition;
    int integralSum=0;
    boolean brake=false;

    //define kP, kI, and kD by actually testing
    //double kP=2.5;
    //double kI=0.025;
    //double kD=0.025;

    ElapsedTime timer;

    public LiftAsync(HardwareMap hwMap, int target)
    {
        this.motor1=hwMap.get(DcMotorEx.class, "liftLeft");
        this.motor2=hwMap.get(DcMotorEx.class, "liftRight");

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        targetPos=target;
        timer=new ElapsedTime();
    }

    public void setPosition(int target)
    {
        //resetLiftTimer();
        brake=false;
        targetPos=target;
        timer.reset();
        previousPosition=Math.abs(motor1.getCurrentPosition());
    }

    public void adjustLift()
    {
        if(!brake)
        {
            error = targetPos - Math.abs(motor1.getCurrentPosition());
            motorPower = Math.signum(error)*(0.7 + 0.3 * (Math.abs(error) / 300));
            motor1.setPower(-motorPower);
            motor2.setPower(motorPower);
        }
        else
            {
                motor1.setPower(0);
                motor2.setPower(0);
            }
        /*currentPos=Math.abs(motor2.getCurrentPosition());

        error=targetPos-currentPos;
        integralSum+=error*timer.seconds();
        derivative=(error-previousError)/timer.seconds();

        motorPower=(error*kP)+(integralSum*kI)+(derivative*kD)/100;
        motor1.setPower(-motorPower);
        motor2.setPower(motorPower);

        previousError=error;
        timer.reset();*/
    }
    public boolean isBusy()
    {
        if(targetPos==0&Math.abs(motor1.getCurrentPosition())<10)
        {
            return false;
        }
        else if(Math.abs(motor1.getCurrentPosition())>targetPos)
        {
            return false;
        }
        return true;
    }
    public void brake()
    {
        brake=true;
    }
    public double currentPower()
    {
        return motorPower;
    }

    public void setPower(double pow)
    {
        motor1.setPower(pow);
        motor2.setPower(pow);
    }
    public boolean stalling()
    {
        if(timer.milliseconds()>750)
        {
            if(Math.abs(Math.abs(motor1.getCurrentPosition())-previousPosition)<15)
            {
                timer.reset();
                previousPosition=Math.abs(motor1.getCurrentPosition());
                return true;
            }
            else
                {
                    timer.reset();
                    previousPosition=Math.abs(motor1.getCurrentPosition());
                }
        }
        return false;
    }

    public int getPos1()
    {
        return motor1.getCurrentPosition();
    }
    public int getPos2()
    {
        return motor2.getCurrentPosition();
    }
}
