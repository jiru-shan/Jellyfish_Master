package org.firstinspires.ftc.teamcode.past_code;

import com.qualcomm.robotcore.util.Range;

//Controls motor powers. Incomplete

public class BasicController extends Controller {
    @Override
    public double getDriveMotorPowers(double reference, double state, double maxSpeed) {
        return 0;
    }

    @Override
    public double getTurnMotorPowers(double reference, double state, double maxSpeed, double angle)
    {

        double error=Math.abs(reference-state)%180;
        double output=((error/Math.abs(angle)))*maxSpeed+2*((1/error)*maxSpeed);
        output*=Math.signum(angle);
        return Range.clip(output, -1, 1);
        /*double error=reference-state;
        double output=((error/Math.abs(angle)))*maxSpeed+2*((1/error)*maxSpeed);
        return Range.clip(output, -1, 1);
*/
    }
}
