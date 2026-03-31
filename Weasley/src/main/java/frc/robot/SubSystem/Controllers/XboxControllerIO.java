package frc.robot.SubSystem.Controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxControllerIO implements ControllerIO{

    XboxController xbox;
    //true means on, false means off
    private boolean startShooter = false;
    private boolean startShooterInverted = false;

    public XboxControllerIO(int port) {
        xbox = new XboxController(port);
        
    }

    @Override
    public double getDriveX() {
        return MathUtil.applyDeadband(xbox.getLeftY(), 0.1);//-xbox.getLeftX();
    }

    @Override
    public double getDriveY() {
        return MathUtil.applyDeadband(xbox.getLeftX(), 0.1);//-xbox.getLeftY();
    }

    @Override
    public double getDriveTwist() {
        return MathUtil.applyDeadband(-xbox.getRightX(), 0.1);//xbox.getRightX();
    }

    @Override
    public boolean startShooter() {
        if (xbox.getRightBumperButtonPressed()) startShooter = startShooter? false:true;
        return xbox.getRightBumperButton();
    }

    @Override
    public boolean startShooterInverted() {
        if (xbox.getLeftBumperButtonPressed()) startShooterInverted = startShooterInverted?false:true;
        return startShooterInverted;
    }

    @Override
    public boolean hopperOut() {
        return xbox.getRightTriggerAxis() > 0.5;
    }

    @Override
    public boolean hopperIn() {
        return xbox.getLeftTriggerAxis() > 0.5;
    }

    @Override
    public boolean climbUp() {
        return xbox.getPOV() == 0;
    }

    @Override
    public boolean climbDown() {
        return xbox.getPOV() ==180;
    }





}
