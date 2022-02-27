package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config

public class ServoControl
{
    HardwareMap hwMap;

    Servo i_topLeft, i_bottomLeft, i_topRight, i_bottomRight, d_open, d_arm, d_turret;

    enum Side
        {
            LEFT, RIGHT
        };
    Side side;

    double i_minRange_topLeft = 0.15;
    double i_maxRange_topLeft = 0.96;
    double i_minRange_bottomLeft = 0.90;
    double i_maxRange_bottomLeft = 0.09;
    double i_minRange_topRight = 0.96;
    double i_maxRange_topRight = 0.15;
    double i_minRange_bottomRight = 0.15;
    double i_maxRange_bottomRight = 0.96;

    double d_open_center=0.48;
    double d_open_partial_left=0.56;
    public static double d_open_left=0.2;
    double d_open_partial_right=0.36;
    public static double d_open_right=0.76;
    public static double d_open_deposit_left=0.91;
    public static double d_open_deposit_right=0.0;

    double d_arm_default=0.9;
    double d_arm_middle_level=0.85;
    public static double d_arm_medium=0.72;
    public static double d_arm_out=0.30;


    public static double d_turret_center=0.52;
    double d_turret_open_left=0.32;
    public static double d_turret_open_right=0.72;

    public ServoControl(HardwareMap map, Side side)
    {
        hwMap=map;
        i_topLeft = hwMap.servo.get("i_topLeft");
        i_bottomLeft = hwMap.servo.get("i_bottomLeft");
        i_topRight = hwMap.servo.get("i_topRight");
        i_bottomRight = hwMap.servo.get("i_bottomRight");

        d_turret=hwMap.servo.get("turret");
        d_open = hwMap.servo.get("bucket");
        d_arm = hwMap.servo.get("arm");
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
    public void lowerAllIntakes()
    {
        i_topLeft.setPosition(i_minRange_topLeft);
        i_bottomLeft.setPosition(i_minRange_bottomLeft);
        i_topRight.setPosition(i_minRange_topRight);
        i_bottomRight.setPosition(i_minRange_bottomRight);
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
    public void raiseAllIntakes()
    {
        i_topLeft.setPosition(i_maxRange_topLeft);
        i_bottomLeft.setPosition(i_maxRange_bottomLeft);
        i_topRight.setPosition(i_maxRange_topRight);
        i_bottomRight.setPosition(i_maxRange_bottomRight);
    }
    public void startingPos()
    {
        //raiseAllIntakes();
       // d_arm.setPosition(d_arm_default);
        d_turret.setPosition(d_turret_center);
        closeDeposit();
    }
    public void returnArm()
    {
        d_arm.setPosition(d_arm_default);
    }
    public void openDepositIntake()
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
    public void openDeposit()
    {
        if(side==Side.LEFT)
        {
            d_open.setPosition(d_open_deposit_left);
        }
        else if(side==Side.RIGHT)
        {
            d_open.setPosition(d_open_deposit_right);
        }
    }
    public void prepDeposit()
    {
        if(side==Side.LEFT)
        {
            d_open.setPosition(d_open_partial_left);
        }
        else if(side==Side.RIGHT)
        {
            d_open.setPosition(d_open_partial_right);
        }
    }
    public void openTurret()
    {
        if(side==Side.LEFT) {
            d_turret.setPosition(d_turret_open_left);
        }
        else if(side==Side.RIGHT)
        {
            d_turret.setPosition(d_turret_open_right);
        }
    }
    public void flipMiddle_level()
    {
        d_arm.setPosition(d_arm_middle_level);
    }
    public void flipMedium()
    {
        d_arm.setPosition(d_arm_medium);
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
