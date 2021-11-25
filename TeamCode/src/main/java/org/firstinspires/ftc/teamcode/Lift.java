package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class Lift extends SingleMotorMechanism{

    private static final int[] ARM_POSITIONS={0, 100, 200}; //temp values

    public Lift(DcMotor motor)
    {
        super(motor);
    }

    public void moveToLevel(int level, double speed)
    {
        moveToPos(ARM_POSITIONS[Range.clip(level-1, 0, ARM_POSITIONS.length-1)], speed);
    }

}
