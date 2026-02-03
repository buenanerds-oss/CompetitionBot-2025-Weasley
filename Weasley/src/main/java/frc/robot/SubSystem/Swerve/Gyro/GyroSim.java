package frc.robot.SubSystem.Swerve.Gyro;

import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroSim implements GyroIO {

    GyroSimulation gyro;

    public GyroSim() {}

    public GyroSim(GyroSimulation gyro) {
        this.gyro = gyro;
    }

    @Override
    public double getAngleRad() {
        return gyro.getGyroReading().getRadians();
    }

}
