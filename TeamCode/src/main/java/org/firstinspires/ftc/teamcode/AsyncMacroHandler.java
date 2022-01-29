package org.firstinspires.ftc.teamcode;

public class AsyncMacroHandler
{
    LiftAsync lift;
    enum MacroState {gamepad1_lb, gamepad1_rb, gamepad2_dpu, gamepad2_dpr, gamepad2_dpd, gamepad2_dpl,
        gamepad2_x, gamepad2_y, gamepad2_b, gamepad2_a, idle}
    MacroState state;

    enum RobotState
    {

    }

    public void setState(MacroState state)
    {
        this.state=state;
    }

    public boolean isBusy()
    {
        if(state==MacroState.idle)
        {
            return false;
        }
        return true;
    }

    public void execute()
    {
        switch (state)
        {
            case gamepad1_lb:
                gamepad1lb();
                break;
            case gamepad1_rb:
                gamepad1rb();
                break;
            case gamepad2_dpu:
                gamepad2dpu();
                break;
            case gamepad2_dpr:
                gamepad2dpr();
                break;
            case gamepad2_dpd:
                gamepad2dpd();
                break;
            case gamepad2_dpl:
                gamepad2dpl();
                break;
            case gamepad2_x:
                gamepad2x();
                break;
            case gamepad2_y:
                gamepad2y();
                break;
            case gamepad2_b:
                gamepad2b();
                break;
            case gamepad2_a:
                gamepad2a();
                break;
            case idle:
                break;
        }
        lift.adjustLift();
    }

    private void gamepad1lb()
    {

    }
    private void gamepad1rb()
    {

    }
    private void gamepad2dpu()
    {

    }
    private void gamepad2dpr()
    {

    }
    private void gamepad2dpd()
    {

    }
    private void gamepad2dpl()
    {

    }
    private void gamepad2x()
    {

    }
    private void gamepad2y()
    {

    }
    private void gamepad2b()
    {

    }
    private void gamepad2a()
    {

    }

}
