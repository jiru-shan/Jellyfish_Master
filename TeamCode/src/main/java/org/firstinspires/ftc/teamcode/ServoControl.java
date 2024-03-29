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
    double i_minRange_topRight = 0.90;
    double i_maxRange_topRight = 0.09;
    double i_minRange_bottomRight = 0.16;
    double i_maxRange_bottomRight = 0.97;

    double d_open_center=0.48;
    public static double d_open_partial_left=0.5;
    public static double d_open_left=0.17;
    public static double d_open_partial_right=0.46;
    public static double d_open_right=0.79;
    public static double d_open_deposit_left=0.72;
    public static double d_open_deposit_right=0.21;
    public static double d_open_middle_left=0.9;
    public static double d_open_middle_right;
    public static double d_open_lowest_left=0.01;
    public static double d_open_lowest_right=0.01;

    double d_arm_default=0.91;
    public static double d_arm_middle_level=0.99;
    public static double d_arm_medium=0.72;
    public static double d_arm_out=0.30;
    public static double d_arm_lowest=1;

    public static double d_turret_center=0.52;
    public static double d_turret_open_left=0.25;
    public static double d_turret_open_right=0.76;
    public static double d_turret_lowest_left=0.99;
    public static double d_turret_lowest_right=0.99;

    //lift:

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
    public void openDepositMiddleLevel()
    {
        if(side==Side.LEFT)
        {
            d_open.setPosition(d_open_middle_left);
        }
        else if(side==Side.RIGHT)
        {
            d_open.setPosition(d_open_middle_right);
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
    public void openTurretLow()
    {
        if(side==Side.LEFT) {
            d_turret.setPosition(d_turret_lowest_left);
        }
        else if(side==Side.RIGHT)
        {
            d_turret.setPosition(d_turret_lowest_right);
        }
    }
    public void openDepositLow()
    {
        if(side==Side.LEFT)
        {
            d_open.setPosition(d_open_lowest_left);
        }
        else if(side==Side.RIGHT)
        {
            d_open.setPosition(d_open_lowest_right);
        }
    }
    public void flipLowest()
    {
        d_arm.setPosition(d_arm_lowest);
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
