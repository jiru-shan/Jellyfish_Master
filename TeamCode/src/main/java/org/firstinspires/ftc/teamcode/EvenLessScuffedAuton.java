package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class EvenLessScuffedAuton extends LinearOpMode
{
    DcMotorEx test;
    private DistanceSensor sensorRange1, sensorRange2;
    @Override
    public void runOpMode() throws InterruptedException
    {
        test.getCurrent(CurrentUnit.AMPS);
        sensorRange1 = hardwareMap.get(DistanceSensor.class, "sensor_range");
        sensorRange2 = hardwareMap.get(DistanceSensor.class, "sensor_range");
        SampleMecanumDrive drive=new SampleMecanumDrive(hardwareMap);
        Pose2d startPose=new Pose2d(0,0,0);
        ElapsedTime time=new ElapsedTime();
        drive.setPoseEstimate(startPose);
        //TrajectorySequence traqSequence=drive.trajectorySequenceBuilder(startPose).


    }

    public boolean hasBlock() {
        if (sensorRange1.getDistance(DistanceUnit.MM) < 10 || sensorRange2.getDistance(DistanceUnit.MM) < 10) {
            return true;
        }
        return false;

    }
}
