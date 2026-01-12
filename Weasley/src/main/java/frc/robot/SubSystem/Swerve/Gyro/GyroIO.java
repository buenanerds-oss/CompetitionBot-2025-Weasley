package frc.robot.SubSystem.Swerve.Gyro;

public interface GyroIO {
    public default double getAngleRad() {
        return 0.0;
    }
}
