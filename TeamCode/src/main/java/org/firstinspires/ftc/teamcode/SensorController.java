package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cControllerPortDeviceImpl;
import com.qualcomm.robotcore.hardware.I2cDeviceImpl;
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
        driveLeft=hwMap.get(ColorRangeSensor.class, "driveLeft");
        driveRight=hwMap.get(ColorRangeSensor.class, "driveRight");
        depositSensor=hwMap.get(ColorRangeSensor.class, "bucketSensor");

        depositTimer=new ElapsedTime();
    }

    public boolean hasBlock()
    {

        if (intakeRange.getDistance(DistanceUnit.CM) < 14)
        {
            return true;
        }
        return false;
    }
    public double intakeDistance()
    {
        return intakeRange.getDistance(DistanceUnit.CM);
    }

    public boolean onColor()
    {
        if(driveLeft.alpha()>450||driveRight.alpha()>450)
        {
            return true;
        }
        return false;
    }

    public boolean depositCube()
    {
        updateDepositValue();
        if(depositValue<85)
        {
            return true;
        }
        return false;
    }
    public boolean depositNoCube()
    {
        if(depositSensor.getDistance(DistanceUnit.MM)>500)
        {
            return true;
        }
        else
            {
                return false;
            }
    }
    private void updateDepositValue()
    {
        if(depositTimer.milliseconds()>300)
        {
            depositValue=depositSensor.getDistance(DistanceUnit.MM);
            depositTimer.reset();
        }
    }

    public double[] getData() {
        updateDepositValue();
        double[] pain = {intakeRange.getDistance(DistanceUnit.CM), depositValue, 0, 0, driveLeft.alpha(), driveRight.alpha()};
        return pain;
    }
    public void closeIntakeSensor()
    {
        intakeRange.close();
    }
    public boolean test()
    {
        if(intakeRange.getDistance(DistanceUnit.CM)<10000)
        {
            return true;
        }
        return false;
    }

}
