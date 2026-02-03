package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.SubSystem.Controllers.JoystickIO;
import frc.robot.SubSystem.Swerve.Drive;

public class DriveCommand {

    
    double IncrementPID;
    public DriveCommand() {
        this.IncrementPID = 0.0;
    }

    public Command JoystickDrive(Drive drive, double x, double y, double rot) {
        return Commands.run(() -> drive.move(x, y, rot), drive);
        
    }

    /**
     * 
     * @param increase - true to increase, false to decrease
     * @return
     */
    public Command ChangePID(Drive drive,String mode, boolean increase) {
        return new Command() {
            @Override
            public void execute() {
                if (!mode.equalsIgnoreCase("increment")) IncrementPID += increase? 0.1: -0.1;
                else drive.changePID(mode, increase? IncrementPID : -IncrementPID);
            }
        };
    }

}
