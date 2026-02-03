package frc.robot.SubSystem.Swerve;

import java.util.Optional;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO{

    /**
     * moves the modules
     * @param desiredState - the state that you want the module to be in
     */
    public default void setDesiredSwerveState(SwerveModuleState desiredState) {

    }

    public default void forceSetVoltage(double turnVolts, double driveVolts) {

    }
    /**
     * changes the PID for the turn controller
     * @param valueToChange - P, I, D, only. no Variation allowed either
     * @param IncrementAmount - set negative to reduce value
     */
    public default void changeModuleTurnPID(String valueToChange, double IncrementAmount) {

    }

     /**
     * changes the PID for the drive controller
     * @param valueToChange - P, I, D, only. no Variation allowed either
     * @param IncrementAmount - set negative to reduce value
     */
    public default void changeModuleDrivePID(String valueToChange, double IncrementAmount) {

    }

    /**logs the individual module */
    public default void periodic() {

    }

    public default SwerveModulePosition getmodulePosition(){
        return new SwerveModulePosition();
    }

    public default Optional<SwerveDriveSimulation> getSwerveSim() {
        return Optional.empty();
    }

    

}
