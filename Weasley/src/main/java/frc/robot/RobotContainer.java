// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ResourceBundle.Control;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SubSystem.Climb.Climb;
import frc.robot.SubSystem.Climb.ClimbIO;
import frc.robot.SubSystem.Controllers.ControllerIO;
import frc.robot.SubSystem.Controllers.JoystickIO;
import frc.robot.SubSystem.Controllers.XboxControllerIO;
import frc.robot.SubSystem.FuelControl.FuelControl;
import frc.robot.SubSystem.Logging.GroupLogger;
import frc.robot.SubSystem.Logging.NerdLog;
import frc.robot.SubSystem.Swerve.Drive;
import frc.robot.SubSystem.Swerve.Module;
import frc.robot.SubSystem.Swerve.ModuleIO;
import frc.robot.SubSystem.Swerve.ModuleSIm;
import frc.robot.SubSystem.Swerve.Gyro.GyroIO;
import frc.robot.SubSystem.Swerve.Gyro.GyroSim;
import frc.robot.SubSystem.Swerve.Gyro.Pidgeon2IO;
import frc.robot.SubSystem.Vision.Vision;
import frc.robot.SubSystem.Vision.VisionIO;

public class RobotContainer {
  ControllerIO Controller = new JoystickIO(0);//XboxControllerIO(0);

  final double driveSpeedFactor = 1;


  Drive swerve;
  FuelControl fuelCrtl;
  ClimbIO climb;
  GyroIO gyro;
  VisionIO vision;

  public RobotContainer() {
    NerdLog.startLog();
   GroupLogger.startGroupLogger();
   
   ModuleIO[] modules = new ModuleIO[4];
    if (Robot.isReal()) {
      for (int i = 0; i <= 3; i++) {
        modules[i] = new Module(i, RobotMap.SwerveTurnMotors[i], RobotMap.SwerveDriveMotors[i]);
      }
      this.gyro = new Pidgeon2IO();
      this.vision = new Vision(new PhotonCamera[] {RobotMap.c270Cam, RobotMap.nexigoCam}, RobotMap.robotToCameras, swerve);
    }
    else {
      for (int i = 0; i<=3; i++) {
        modules[i] = new ModuleSIm(i);
      }
      if (modules[0].getSwerveSim().isPresent()) this.gyro = new GyroSim(modules[0].getSwerveSim().get().getGyroSimulation());
      else this.gyro = new GyroSim();
      this.vision = new VisionIO() {};
    }
    
    swerve = new Drive(gyro, modules);
    fuelCrtl = new FuelControl(RobotMap.shooterMotor, RobotMap.hopperMotor);
    climb = new Climb(RobotMap.climbMotor);
    AutoPicker.supplySubSystems(swerve, fuelCrtl,climb, vision, RobotMap.robotToCameras);
  }

  

  public void roboPeriodic() {
    swerve.periodic();
    fuelCrtl.periodic();
    climb.periodic();
    vision.periodic();
  }

  public void enabled() {
    
    NerdLog.logDouble("joystick X", Controller.getDriveX());
    NerdLog.logDouble("joystick y", Controller.getDriveY());
    swerve.move(Controller.getDriveX(), Controller.getDriveY(), Controller.getDriveTwist());
    if (Controller.startShooter()) fuelCrtl.shootShooter();
    else if (Controller.startShooterInverted()) fuelCrtl.shootShooterInverted();
    else {fuelCrtl.stopShooting();}
    if (Controller.hopperOut()) fuelCrtl.outtake();
    else if (Controller.hopperIn()) fuelCrtl.intake();
    else fuelCrtl.stopHopper();
    if (Controller.climbUp()) climb.climbUp();
    else if (Controller.climbDown()) climb.climbDown();
    else climb.stop();
    if (Controller.resetGyro()) gyro.reset();
    
  }

  public void disabledPeriodic() {

  }



  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
