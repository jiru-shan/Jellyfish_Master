package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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
    double baseVelocity;

    //define kP, kI, and kD by actually testing
    //double kP=2.5;
    //double kI=0.025;
    //double kD=0.025;

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
        motorPower=1000;
    }

    public void setPosition(int target)
    {
        brake=false;
        targetPos=target;
        timer.reset();
        previousPosition=Math.abs(motor1.getCurrentPosition());
        motor1.setTargetPosition(targetPos);
        motor2.setTargetPosition(targetPos);
        motorPower=1;
        //motorPower=1000;
    }
    public void setPosition(int target, double vel)
    {
        brake=false;
        targetPos=target;
        timer.reset();
        previousPosition=Math.abs(motor1.getCurrentPosition());
        motor1.setTargetPosition(targetPos);
        motor2.setTargetPosition(targetPos);
        motorPower=vel;
    }
    public double getVelocity()
    {
        //motor1.setVelocity(300);
        return motor1.getVelocity(AngleUnit.DEGREES);
    }
    public void adjustLift()
    {
        int dir;
        if(targetPos==0)
        {
            dir=-1;
        }
        else
            {
                dir=1;
            }

        updateBrake();
        if(!brake)
        {
            //motor1.get
            //motor1.setVelocity(dir*motorPower);
            //motor2.setVelocity(dir*motorPower);
            motor1.setPower(dir*motorPower);
            motor2.setPower(dir*motorPower);
        }
        else
            {
                motor1.setVelocity(0);
                motor2.setVelocity(0);
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
        return !brake;
    }
    public void updateBrake()
    {
        if(targetPos==0)
        {
            if(Math.abs(motor2.getCurrentPosition())<10)
            {
                brake=true;
            }
        }
        else if(Math.abs(motor2.getCurrentPosition())>targetPos-5)
        {
            brake=true;
        }
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
    public double getPower()
    {
        return motor1.getPower();
       // return motorPower;
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
