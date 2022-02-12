package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@TeleOp
public class LiftTest extends LinearOpMode
{
    LiftAsync lift;
    ElapsedTime timer;
    static int position;

    @Override
    public void runOpMode() throws InterruptedException
    {
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
