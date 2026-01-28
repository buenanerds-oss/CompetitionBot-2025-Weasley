package frc.robot.SubSystem.Controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.SubSystem.Logging.NerdLog;

public class JoystickIO implements ControllerIO{
    Joystick joystick;
    boolean PidPControlwasPressed = false;
    boolean PidDControlwasPressed = false;
    boolean PidIncrementControlwasPressed = false;
    double PIDIncrementAmount = 0;


    public JoystickIO(int port) {
        this.joystick = new Joystick(port);
    }

    @Override
    public double getDriveX() {
        return -joystick.getY();
    }

    @Override
    public double getDriveY() {
        return -joystick.getX();
    }

    @Override
    public double getDriveTwist() {
        return joystick.getTwist(); // TODO invert if needed;
    }

    @Override
    public double getPIDPChange() {

        NerdLog.logDouble("PID Increment Amount", PIDIncrementAmount);
        if (joystick.getRawButtonPressed(11) && !PidIncrementControlwasPressed) {
            PidIncrementControlwasPressed = true;
            PIDIncrementAmount += 0.01;
        }
        else if (joystick.getRawButton(12)) {
            PidIncrementControlwasPressed = true;
            PIDIncrementAmount +=0.01;
        }
        else if (PidIncrementControlwasPressed) {
        }
        else {
            PidIncrementControlwasPressed = false;
        }

        if (PidPControlwasPressed && !PidPControlwasPressed) return 0.00;
        else if(joystick.getRawButtonPressed(9) ) {
            PidPControlwasPressed = true;
            return PIDIncrementAmount;
        }
        else if(joystick.getRawButtonPressed(10) && !PidPControlwasPressed)  {
            PidPControlwasPressed = true;
            return -1.00 * PIDIncrementAmount;
        }
        else if (PidPControlwasPressed) return 0.00;
        PidPControlwasPressed = false;
        return 0.00;
    }

    @Override
    public double getPIDDChange() {

        
        if(joystick.getRawButtonPressed(13) && !PidDControlwasPressed) {
            PidDControlwasPressed = true;
            return PIDIncrementAmount;
        }
        else if(joystick.getRawButtonPressed(14) && !PidDControlwasPressed) {
            PidDControlwasPressed = true;
            return -1.00 * PIDIncrementAmount;
        }
        else if (PidDControlwasPressed) return 0.00;
        // the rest is basically else{
        PidDControlwasPressed = false;
        return 0.00;
    }
    
}
