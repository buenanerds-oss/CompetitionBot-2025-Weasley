// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.DriveCommand;
import frc.robot.SubSystem.Controllers.ControllerIO;
import frc.robot.SubSystem.Controllers.XboxControllerIO;
import frc.robot.SubSystem.Logging.GroupLogger;
import frc.robot.SubSystem.Logging.NerdLog;
import frc.robot.SubSystem.Swerve.Drive;
import frc.robot.SubSystem.Swerve.Module;
import frc.robot.SubSystem.Swerve.ModuleIO;
import frc.robot.SubSystem.Swerve.ModuleSIm;
import frc.robot.SubSystem.Swerve.Gyro.GyroIO;
import frc.robot.SubSystem.Swerve.Gyro.GyroSim;
import frc.robot.SubSystem.Swerve.Gyro.Pidgeon2IO;

public class RobotContainer {
  //ControllerIO controller = new JoystickIO(0);
  ControllerIO Controller = new XboxControllerIO(0);

  final double driveSpeedFactor = 1;
  String[] PIDModes = {"P", "I", "D"};
  int PIDModeSelection = 0;


  Drive swerve;
  DriveCommand driveCmd;

  public RobotContainer() {
    NerdLog.startLog();
    GroupLogger.startGroupLogger();

    ModuleIO[] modules = new ModuleIO[4];
    GyroIO gyro;
    if (Robot.isReal()) {
      for (int i = 0; i <= 3; i++) {
        modules[i] = new Module(i, RobotMap.SwerveTurnMotors[i], RobotMap.SwerveDriveMotors[i]);
      }
      gyro = new Pidgeon2IO();
    }
    else {
      for (int i = 0; i<=3; i++) {
        modules[i] = new ModuleSIm(i);
      }
      if (modules[0].getSwerveSim().isPresent()) gyro = new GyroSim(modules[0].getSwerveSim().get().getGyroSimulation());
      else gyro = new GyroSim();
    }
    
    swerve = new Drive(gyro, modules);
    driveCmd = new DriveCommand();

    configureControls();

  }

  

  public void roboPeriodic() {
    swerve.periodic();
    NerdLog.logDouble("PID Mode Selection", PIDModeSelection);
  }

  public void disabledPeriodic() {

    //PID changing
   Controller.getPIDIncrease().onTrue(driveCmd.ChangePID(swerve, PIDModes[PIDModeSelection], true));
   Controller.getPIDDecrease().onTrue(driveCmd.ChangePID(swerve, PIDModes[PIDModeSelection], false));

   //PID Mode switching
   Controller.getPIDSwitchPositive().onTrue(Commands.runOnce( () -> {
    if (PIDModeSelection < PIDModes.length) PIDModeSelection++;
    else PIDModeSelection = 0;
   }));
   Controller.getPIDSwitchNegative().onTrue(Commands.runOnce(() -> {
    if (PIDModeSelection >= 0) PIDModeSelection--;
    else PIDModeSelection = 3;
   }));

  }

  private void configureControls() {
    swerve.setDefaultCommand(driveCmd.JoystickDrive(swerve, Controller.getDriveX() * driveSpeedFactor,
    Controller.getDriveY() *  driveSpeedFactor, 
    Controller.getDriveTwist() * driveSpeedFactor));
    
  }



  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
