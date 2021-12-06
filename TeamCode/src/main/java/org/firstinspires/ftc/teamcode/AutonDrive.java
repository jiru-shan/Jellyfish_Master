package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;
import java.util.List;

//Object representing the drivetrain

public class AutonDrive extends MecanumDrivetrain
{
    //Options for turning
    //Current method which is buggy and sometimes dies/twitches around(+=180 for normalize and BasicController)
    //Modified method which probably wont work(%360 for normalize and modified BasicController)
    //ImuSensor Method
    //Oldest method. Use tempfactor and 0.2+(error/angle)*0.5

    BNO055IMU imu;
    static final double     COUNTS_PER_MOTOR_REV    = 420 ;//base is 28(or is it 14? have to test) and then gearbox is 15 so 28*15    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP //drivetrain motors have 2.89:1 and 5.23:1 so either 15.1147:1 or 1:15.1147
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double MARGIN_OF_ERROR=3;
    static final double STRAFE_THRESHOLD=10;
    static final double DRIVE_COEFF=0.15;
    static final double ERROR=10;
    static final double ACCEL_THRESHHOLD=3;
    Telemetry tel;

    Controller control;

    private double targetAngle;
    public AutonDrive(DcMotor[] motors, BNO055IMU imu, Telemetry tel, Controller control)
    {
        super(motors);
        this.imu=imu;
        this.tel=tel;
        this.control=control;
    }

    public void moveInches(double distance, double speed, double angle, double timeout)
    {
        speed=Range.clip(speed, -1, 1);
        double factor=distance/Math.abs(distance);

        double currentPos=getPosition();
        int target=(int) (currentPos+ (distance*COUNTS_PER_INCH));
        double timedout= SystemClock.uptimeMillis()+(timeout*1000);
        while(Math.abs(target-getPosition())>ERROR&&SystemClock.uptimeMillis()<timedout)
        {
            for(DcMotor i:motors)
            {
                i.setPower(speed*factor);
            }
        }

        for(DcMotor i:motors)
        {
            i.setPower(0);
        }
    }
    /* public void moveInches(double inches, double speed, double angle)
     {
         double error;
         double steer;
         targetHeading=getCurrentHeading()+angle;
         double leftSpeed=speed;
         double rightSpeed=speed;


         for(DcMotor i:motors){i.setTargetPosition(i.getCurrentPosition()+(int) (inches*COUNTS_PER_INCH));}
         for(DcMotor i: motors){i.setMode(DcMotor.RunMode.RUN_TO_POSITION);}
         for(DcMotor i: motors){i.setPower(Range.clip(Math.abs(speed), -1, 1));}

         while(motors[0].isBusy()&&motors[1].isBusy())
         {
             error=getError();
             steer=error*DRIVE_COEFF;

             if (inches < 0)
                 steer *= -1.0;

             leftSpeed-=steer;
             rightSpeed+=steer;

             motors[0].setPower(leftSpeed);
             motors[1].setPower(leftSpeed);
             motors[2].setPower(rightSpeed);
             motors[3].setPower(rightSpeed);


         }

         for(DcMotor i:motors){i.setPower(0);}
         for(DcMotor i:motors){i.setMode(DcMotor.RunMode.RUN_USING_ENCODER);}
     }*/
    public void turnDegrees(double angle, double speed, double timeout)
    {
        targetAngle=normalizeAngle(getCurrentHeading()+angle);
        speed=Range.clip(speed, -1,1);
        double timedout= SystemClock.uptimeMillis()+(timeout*1000);
        while(Math.abs(getCurrentHeading()-targetAngle)>MARGIN_OF_ERROR&&SystemClock.uptimeMillis()<timedout)
        {
            //tel.addData("Target", targetAngle);
            //tel.addData("Present",  getCurrentHeading());
            //tel.update();
            double tempSpeed=control.getTurnMotorPowers(targetAngle, getCurrentHeading(), speed, angle);
            motors[3].setPower(tempSpeed);
            motors[1].setPower(-tempSpeed);
            motors[2].setPower(tempSpeed);
            motors[0].setPower(-tempSpeed);
        }
        for(DcMotor i:motors){i.setPower(0);}
    }
    public void turnToDegrees(double angle, double speed, double timeout)
    {
        targetAngle=normalizeAngle(angle);
        angle=targetAngle-getCurrentHeading();
        speed=Range.clip(speed, -1,1);
        double factor=Math.signum(angle);
        double timedout= SystemClock.uptimeMillis()+(timeout*1000);
        while(Math.abs(getCurrentHeading()-targetAngle)>2&&SystemClock.uptimeMillis()<timedout)
        {
            tel.addData("Target", targetAngle);
            tel.addData("Present",  getCurrentHeading());
            tel.update();
            //double tempSpeed=control.getTurnMotorPowers(targetAngle, getCurrentHeading(), speed, angle);
            double error=getCurrentHeading()-targetAngle;
            double tempSpeed=0.3+(0.4*Math.abs(error/angle));
            motors[3].setPower(tempSpeed*factor);
            motors[1].setPower(-tempSpeed*factor);
            motors[2].setPower(tempSpeed*factor);
            motors[0].setPower(-tempSpeed*factor);
        }
        for(DcMotor i:motors){i.setPower(0);}
    }
    public void recheckHeading(double speed, double timeout)
    {

        double angle=targetAngle-getCurrentHeading();
        speed=Range.clip(speed, -1, 1);
        double timedout= SystemClock.uptimeMillis()+(timeout*1000);
        while(Math.abs(getCurrentHeading()-targetAngle)>MARGIN_OF_ERROR&&SystemClock.uptimeMillis()<timedout)
        {
            //tel.addData("Target", targetAngle);
            //tel.addData("Present",  getCurrentHeading());
            //tel.update();
            double tempSpeed=control.getTurnMotorPowers(targetAngle, getCurrentHeading(), speed, angle);
            motors[3].setPower(tempSpeed);
            motors[1].setPower(-tempSpeed);
            motors[2].setPower(tempSpeed);
            motors[0].setPower(-tempSpeed);
        }
        for(DcMotor i:motors){i.setPower(0);}
    }
    public double normalizeAngle(double angle)
    {
        /*double output=angle;
        while(angle<-180)
        {
            output+=360;
        }
        while(angle>180)
        {
            output-=360;
        }*/

        return (angle+360)%360;
    }
    public double getCurrentHeading()
    {
        return normalizeAngle(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
    }
    public double getPosition()
    {
        double temp=0;
        for(DcMotor i:motors)
        {
            temp+=i.getCurrentPosition();
        }
        temp/=motors.length;
        return temp;
    }
    public void movePipes(double speed, double direction, double timeout)
    {
        moveInches(Math.signum(direction)*30, speed, 0, 3 );
        double timedout=SystemClock.uptimeMillis()+(timeout)*1000;
        while(SystemClock.uptimeMillis()<timedout&&Math.abs(imu.getLinearAcceleration().yAccel)<ACCEL_THRESHHOLD)
        {
            for(DcMotor i:motors)
            {
                i.setPower(speed*Math.signum(direction));
            }
        }

        for(DcMotor i:motors)
        {
            i.setPower(0);
        }


        /*List<Double> previousAccel=new ArrayList<>();
        double timedout= SystemClock.uptimeMillis()+(timeout*1000);
        while(previousAccel.get(previousAccel.size()-1)/previousAccel.get(0)-1<0.5&&SystemClock.uptimeMillis()<timedout)
        {
            for(DcMotor i: motors)
            {
                i.setPower(speed*Math.signum(direction));
            }
            previousAccel.add(imu.getAcceleration().xAccel);
            if(previousAccel.size()>10)
            {
                previousAccel.remove(0);
            }

        }
        for(DcMotor i:motors)
        {
            i.setPower(0);
        }
    }*/
    }
}
