package frc.robot.SubSystem.Swerve;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import frc.robot.SubSystem.Logging.GroupLogger;
import frc.robot.Util.BasicUtil;

public class ModuleSIm implements ModuleIO { // TODO document

    //what i plan to log:
    Pose2d currentPose; 
    SwerveModulePosition currentPosition;
    SwerveModuleState currentState;
    int index;
    double driveVolts;
    double turnVolts;
    boolean emergencyStop;

    SwerveModuleSimulation moduleSim;
    GenericMotorController driveMotor;
    GenericMotorController turnMotor;
    PIDController drivePID;
    PIDController turnPID;

    SwerveModuleSimulationConfig moduleconfig = COTS.ofMark4(DCMotor.getNEO(1), DCMotor.getNEO(1), 1.2, 2);
    DriveTrainSimulationConfig drivetrainConfig = DriveTrainSimulationConfig.Default()
    .withCustomModuleTranslations(SwerveConstants.moduletranslations)
    .withRobotMass(Kilograms.of(SwerveConstants.mass_KG))
    .withGyro(COTS.ofPigeon2())
    .withSwerveModule(moduleconfig);

    public SwerveDriveSimulation driveSim = new SwerveDriveSimulation(drivetrainConfig, SwerveConstants.initialPose);

    public ModuleSIm(int index) {
        //Simulation Initialization
        moduleSim = driveSim.getModules()[index];
        driveMotor = moduleSim.useGenericMotorControllerForDrive();
        turnMotor = moduleSim.useGenericControllerForSteer();

        //initialize loggables
        driveVolts = 0;
        turnVolts = 0; 
        emergencyStop = false;
        currentPosition = new SwerveModulePosition(Units.rotationsToRadians(moduleSim.getDriveEncoderUnGearedPosition().in(Rotation) * SwerveConstants.driveGearRatio) * SwerveConstants.wheelRadiusMeters,
        new Rotation2d(Units.rotationsToRadians(moduleSim.getSteerAbsoluteFacing().getRotations() * SwerveConstants.turnGearRatio)));
        currentState = moduleSim.getCurrentState();


        //Initialize PID
        drivePID = new PIDController(ModuleMotorConfig.DRIVE_P, ModuleMotorConfig.DRIVE_I, ModuleMotorConfig.DRIVE_D);
        turnPID = new PIDController(ModuleMotorConfig.TURN_P, ModuleMotorConfig.TURN_I, ModuleMotorConfig.TURN_D);

        SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);
    }

    @Override
    public void setDesiredSwerveState(SwerveModuleState desiredState) {
        desiredState.optimize(currentPosition.angle); // optimize finds the shortest path for the turn motor to rotate to reach it's direction
        desiredState.cosineScale(currentPosition.angle); //optimize will change the drive values to match the direction the swerve is facing

        if (driveMotor.getAppliedVoltage().in(Volts) >= SwerveConstants.driveMaxVolts || turnMotor.getAppliedVoltage().in(Volts) >= SwerveConstants.turnMaxVolts)
        emergencyStop = true;

        if (!emergencyStop && !BasicUtil.numIsInBallparkOf(currentPosition.angle.getRadians(), desiredState.angle.getRadians(), SwerveConstants.turnAccuracyToleranceRAD)) {
            turnPID.setSetpoint(desiredState.angle.getRadians());
            turnMotor.requestVoltage(Volts.of(turnPID.calculate(currentPosition.angle.getRadians())));
        }
        else { turnMotor.requestVoltage(Volts.of(0));}

        if (!emergencyStop && 
        !BasicUtil.numIsInBallparkOf(currentState.speedMetersPerSecond, desiredState.speedMetersPerSecond, SwerveConstants.driveAccuracyToleranceMPS) &&
        !BasicUtil.numIsInBallparkOf(desiredState.speedMetersPerSecond, 0, SwerveConstants.driveAccuracyToleranceMPS)) {

            drivePID.setSetpoint(desiredState.speedMetersPerSecond);
            driveMotor.requestVoltage(Volts.of(Units.rotationsToRadians(moduleSim.getDriveWheelFinalSpeed().in(RotationsPerSecond) * SwerveConstants.driveGearRatio) * SwerveConstants.wheelRadiusMeters));

        }
        else if (!emergencyStop && !BasicUtil.numIsInBallparkOf(desiredState.speedMetersPerSecond, 0, SwerveConstants.driveAccuracyToleranceMPS)) {
            driveMotor.requestVoltage(Volts.of(driveVolts));
        }
        else { 
            driveMotor.requestVoltage(Volts.of(0));
        }


        //logging:
        driveVolts = driveMotor.getAppliedVoltage().in(Volts);
        turnVolts = turnMotor.getAppliedVoltage().in(Volts);
        currentPosition = new SwerveModulePosition(Units.rotationsToRadians(moduleSim.getDriveWheelFinalPosition().in(Rotations) * SwerveConstants.driveGearRatio),
         new Rotation2d(Units.rotationsToRadians(moduleSim.getSteerAbsoluteFacing().getRotations() * SwerveConstants.turnGearRatio)));
        currentState = moduleSim.getCurrentState();
    }

    @Override
    public void forceSetVoltage(double turnVolts, double driveVolts) {
        driveMotor.requestVoltage(Volts.of(0));
        turnMotor.requestVoltage(Volts.of(0));
    }

    @Override
    public void changeModuleTurnPID(String valueToChange, double IncrementAmount) {
        ModuleMotorConfig.changeTurnSimPID(valueToChange, IncrementAmount);
        turnPID = new PIDController(ModuleMotorConfig.SIM_TURN_P,
         ModuleMotorConfig.SIM_TURN_I,
         ModuleMotorConfig.SIM_TURN_D);
    }

    @Override
    public void changeModuleDrivePID(String valueToChange, double IncrementAmount) {
        ModuleMotorConfig.changeDriveSimPID(valueToChange, IncrementAmount);
        drivePID = new PIDController(ModuleMotorConfig.SIM_DRIVE_P,
         ModuleMotorConfig.SIM_DRIVE_I,
         ModuleMotorConfig.SIM_DRIVE_D);
    }

    @Override
    public void periodic() {

        GroupLogger.logDoubleGroup("Module Drive Voltage", driveVolts, index, 4);
        GroupLogger.logDoubleGroup("Module Turn Voltage", turnVolts, index, 4);
        GroupLogger.LogBooleanGroup("Module Emergency stop", emergencyStop, index, 4);
        GroupLogger.logStructGroup("Swerve Positions", currentPosition, SwerveModulePosition.struct, index, 4);
        GroupLogger.logStructGroup("Swerve States", currentState, SwerveModuleState.struct, index, 4);

        SimulatedArena.getInstance().simulationPeriodic();
    }

    @Override
    public SwerveModulePosition getmodulePosition() {
        return currentPosition;
    }

    @Override
    public Optional<SwerveDriveSimulation> getSwerveSim() {
        return Optional.ofNullable(driveSim);
    }


}
