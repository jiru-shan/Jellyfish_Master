package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SensorController
{
    HardwareMap hwMap;
    ColorRangeSensor intakeRange1, intakeRange2, driveLeft, driveRight, depositSensor;

    
    public SensorController(HardwareMap map)
    {
        hwMap=map;
        intakeRange1 = hwMap.get(ColorRangeSensor.class, "colorSensor_right");
        intakeRange2 = hwMap.get(ColorRangeSensor.class, "colorSensor_left");
        driveLeft=hwMap.get(ColorRangeSensor.class, "driveSensor1");
        driveRight=hwMap.get(ColorRangeSensor.class, "driveSensor2");
        depositSensor=hwMap.get(ColorRangeSensor.class, "bucketSensor");
    }

    public boolean hasBlock()
    {

        if (intakeRange1.getDistance(DistanceUnit.MM) < 55 || intakeRange2.getDistance(DistanceUnit.MM) < 55) {
            return true;
        }
        return false;
    }

    public boolean onColor()
    {
        if(rgbAvg(driveLeft)>175||rgbAvg(driveRight)>175)
        {
            return true;
        }
        return false;
    }

    public boolean depositCube()
    {
        if(depositSensor.getDistance(DistanceUnit.MM)<85)
        {
            return true;
        }
        return false;
    }

    public double[] getData()
    {
        double[] pain={intakeRange1.getDistance(DistanceUnit.MM), intakeRange2.getDistance(DistanceUnit.MM),
                depositSensor.getDistance(DistanceUnit.MM), rgbAvg(driveLeft), rgbAvg(driveRight)};
        return pain;
    }

    private double rgbAvg(ColorRangeSensor pain)
    {
        return (pain.green()+pain.blue()+pain.red())/3;
    }
}
