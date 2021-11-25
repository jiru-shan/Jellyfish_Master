package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class MecanumDrivetrain
{


    DcMotor[] motors;
    //0=FrontLeft, 1=Backleft, 2=FrontRight, 3=BackRight



    public MecanumDrivetrain(DcMotor[] motors)
    {
        this.motors=motors;
        for(DcMotor i: motors)
        {
            i.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //i.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
    }




}
