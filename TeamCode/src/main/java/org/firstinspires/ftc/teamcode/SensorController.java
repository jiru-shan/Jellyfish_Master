package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SensorController
{
    enum Side {LEFT, RIGHT}
    Side side;
    HardwareMap hwMap;
    ColorRangeSensor intakeRange, driveLeft, driveRight, depositSensor;
    ElapsedTime depositTimer;

    double depositValue;
    
    public SensorController(HardwareMap map, Side side)
    {
        this.side=side;
        hwMap=map;
        if(side==Side.LEFT)
        {
            intakeRange = hwMap.get(ColorRangeSensor.class, "colorSensor_left");
        }
        else {
            intakeRange = hwMap.get(ColorRangeSensor.class, "colorSensor_right");
        }
        driveLeft=hwMap.get(ColorRangeSensor.class, "driveSensor1");
        driveRight=hwMap.get(ColorRangeSensor.class, "driveSensor2");
        depositSensor=hwMap.get(ColorRangeSensor.class, "bucketSensor");

        depositTimer=new ElapsedTime();
    }

    public boolean hasBlock()
    {

        if (intakeRange.getDistance(DistanceUnit.MM) < 55)
        {
            return true;
        }
        return false;
    }

    public boolean onColor()
    {
        if(driveLeft.alpha()>400||driveRight.alpha()>400)
        {
            return true;
        }
        return false;
    }

    public boolean depositCube()
    {
        updateDepositValue();
        if(depositValue<100)
        {
            return true;
        }
        return false;
    }
    private void updateDepositValue()
    {
        if(depositTimer.milliseconds()>300)
        {
            depositValue=depositSensor.getDistance(DistanceUnit.MM);
            depositTimer.reset();
        }
    }

    public double[] getData()
    {
        updateDepositValue();
        double[] pain={0, depositValue, driveLeft.alpha(), driveRight.alpha()};
        return pain;
    }
    public void closeIntakeSensor()
    {
        intakeRange.close();
    }

}
