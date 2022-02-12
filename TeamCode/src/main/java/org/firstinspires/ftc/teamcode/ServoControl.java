package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoControl
{
    HardwareMap hwMap;

    Servo i_topLeft, i_bottomLeft, i_topRight, i_bottomRight, d_open, d_arm, d_turret;

    enum Side
        {
            LEFT, RIGHT
        };
    Side side;

    double i_minRange_topRight = 0.87;
    double i_maxRange_topRight = 0.16;
    double i_minRange_bottomRight = 0.16;
    double i_maxRange_bottomRight = 0.87;

    double i_minRange_topLeft = 0.15;
    double i_maxRange_topLeft = 0.87;
    double i_minRange_bottomLeft = 0.9;
    double i_maxRange_bottomLeft = 0.18;

    double d_open_center=0.55;
    double d_open_left=0.27;
    double d_open_right=0.83;

    double d_arm_default=0.87;
    double d_arm_out=0.30;


    double d_turret_center=0.5;

    public ServoControl(HardwareMap map, Side side)
    {
        hwMap=map;
        i_topLeft = hwMap.servo.get("i_topLeft");
        i_bottomLeft = hwMap.servo.get("i_bottomLeft");
        i_topRight = hwMap.servo.get("i_topRight");
        i_bottomRight = hwMap.servo.get("i_bottomRight");

        d_turret=hwMap.servo.get("d_turret");
        d_open = hwMap.servo.get("d_open");
        d_arm = hwMap.servo.get("d_arm");
        this.side=side;
    }

    public void lowerIntakes()
    {
        if(side==Side.LEFT)
        {
            i_topLeft.setPosition(i_minRange_topLeft);
            i_bottomLeft.setPosition(i_minRange_bottomLeft);
        }
        else if(side==Side.RIGHT)
        {
            i_topRight.setPosition(i_minRange_topRight);
            i_bottomRight.setPosition(i_minRange_bottomRight);
        }
    }
    public void raiseIntakes()
    {
        if(side==Side.LEFT)
        {
            i_topLeft.setPosition(i_maxRange_topLeft);
            i_bottomLeft.setPosition(i_maxRange_bottomLeft);
        }
        else if(side==Side.RIGHT)
        {
            i_topRight.setPosition(i_maxRange_topRight);
            i_bottomRight.setPosition(i_maxRange_bottomRight);
        }
    }
    public void startingPos()
    {
        d_arm.setPosition(d_arm_default);
        d_turret.setPosition(d_turret_center);
        closeDeposit();
    }
    public void openDeposit()
    {
        if(side==Side.LEFT)
        {
        d_open.setPosition(d_open_left);
        }
        else if(side==Side.RIGHT)
        {
            d_open.setPosition(d_open_right);
        }
    }
    public void closeDeposit()
    {
        d_open.setPosition(d_open_center);
    }
    public void flipOut()
    {
        d_arm.setPosition(d_arm_out);
    }
}
