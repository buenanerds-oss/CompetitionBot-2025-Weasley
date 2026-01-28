// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.SubSystem.Controllers.ControllerIO;
import frc.robot.SubSystem.Controllers.JoystickIO;
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
  ControllerIO controller = new JoystickIO(0);

  final double driveSpeedFactor = 1;


  Drive swerve;
  public RobotContainer() {
    NerdLog.startLog();
    GroupLogger.startGroupLogger();

    ModuleIO[] modules = new ModuleIO[4];
    GyroIO gyro;
    if (Robot.isReal()) {
      for (int i = 0; i <= 3; i++) {
        modules[i] = new Module(i, AllMotors.SwerveTurnMotors[i], AllMotors.SwerveDriveMotors[i]);
      }
      gyro = new Pidgeon2IO();
    }
    else {
      for (int i = 0; i<=3; i++) {
        modules[i] = new ModuleSIm();
      }
      gyro = new GyroSim();
    }
    
    swerve = new Drive(gyro, modules);

  }

  

  public void doTheDrivingThing() {
    swerve.move(controller.getDriveX(), controller.getDriveY(), controller.getDriveTwist());
    
  }

  public void roboPeriodic() {
    swerve.periodic();
  }

  public void disabledPeriodic() {
    if (Math.abs(controller.getPIDPChange()) > 0 || Math.abs(controller.getPIDDChange()) > 0) {
      swerve.changePID("P", controller.getPIDPChange());
      swerve.changePID("D", controller.getPIDDChange());
    }
  }



  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
