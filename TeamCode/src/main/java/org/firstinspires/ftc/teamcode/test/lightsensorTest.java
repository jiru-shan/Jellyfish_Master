package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous

public class lightsensorTest extends LinearOpMode
{
    ColorRangeSensor sensorRange1, sensorRange2;



    public boolean hasBlock() {

        if (sensorRange1.getDistance(DistanceUnit.MM) < 55 || sensorRange2.getDistance(DistanceUnit.MM) < 55) {
            return true;
        }
        return false;
    }
    public double checkBlock()
    {
        return sensorRange1.getDistance(DistanceUnit.MM);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        sensorRange1 = hardwareMap.get(ColorRangeSensor.class, "colorSensor_right");
        sensorRange2 = hardwareMap.get(ColorRangeSensor.class, "colorSensor_left");

        waitForStart();

        while(opModeIsActive())
        {
            telemetry.addData(">", checkBlock());
            telemetry.addData(">", hasBlock());
            telemetry.update();
        }
    }
}
