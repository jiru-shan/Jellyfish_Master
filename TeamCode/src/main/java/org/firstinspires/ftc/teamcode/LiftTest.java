package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@TeleOp
public class LiftTest extends LinearOpMode
{
    LiftAsync lift;
    ElapsedTime timer;
    public static int position=300;
    DcMotorEx motor1, motor2;

    @Override
    public void runOpMode() throws InterruptedException
    {
        motor1=hardwareMap.get(DcMotorEx.class, "liftLeft");
        motor2=hardwareMap.get(DcMotorEx.class, "liftRight");
        timer=new ElapsedTime();
        lift=new LiftAsync(hardwareMap, 0);
        lift.setPosition(position);
        timer.reset();
        while(!lift.isBusy())
        {
            lift.adjustLift();
            if(timer.milliseconds()>2500)
            {
                requestOpModeStop();
            }
        }

        double target= SystemClock.uptimeMillis()+500;
        while(SystemClock.uptimeMillis()<target)
        {
            lift.adjustLift();
        }

        lift.setPosition(0);
        while(!lift.isBusy())
        {
            lift.adjustLift();
        }
    }
}
