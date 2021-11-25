package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public abstract class SingleMotorMechanism
{
    DcMotor motor;

    public SingleMotorMechanism(DcMotor motor)
    {
        this.motor=motor;
    }
    public void moveToPos(int position, double speed)
    {
        speed= Range.clip(speed, -1, 1);
        double direction=position-motor.getCurrentPosition();
        double factor=Math.abs(direction)/direction;
        while(Math.abs(motor.getCurrentPosition()-position)>10)
        {
            motor.setPower(speed*factor);
        }
        motor.setPower(0);
    }


}
