package frc.robot.SubSystem.Controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxControllerIO implements ControllerIO{

    CommandXboxController xbox;

    public XboxControllerIO(int port) {
        xbox = new CommandXboxController(port);
        
    }

    @Override
    public double getDriveX() {
        return xbox.getLeftX();
    }

    @Override
    public double getDriveY() {
        return xbox.getLeftY();
    }

    @Override
    public double getDriveTwist() {
        return xbox.getRightX();
    }

    @Override
    public Trigger getPIDIncrease() {
        return xbox.povUp();
    }

    @Override
    public Trigger getPIDDecrease() {
        return xbox.povDown();
    }

    @Override
    public Trigger getPIDSwitchPositive() {
        return xbox.povRight();
    }

    @Override
    public Trigger getPIDSwitchNegative() {
        return xbox.povLeft();
    }

}
