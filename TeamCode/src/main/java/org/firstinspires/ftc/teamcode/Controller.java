package org.firstinspires.ftc.teamcode;

public abstract class Controller
{

    public abstract double getDriveMotorPowers(double reference, double state, double maxSpeed);
    public abstract double getTurnMotorPowers(double reference, double state, double maxSpeed, double angle);

}
