package frc.robot.SubSystem.Swerve.Gyro;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.SubSystem.Swerve.SwerveConstants;

public class Pidgeon2IO  implements GyroIO {
    Pigeon2 gyro;
    StatusSignal<Angle> yaw;
    public Pidgeon2IO() {
       gyro = new Pigeon2(SwerveConstants.GyroCANID);

       gyro.getConfigurator().apply(new Pigeon2Configuration());
       gyro.getConfigurator().setYaw(0.0);

       this.yaw = gyro.getYaw();
       yaw.setUpdateFrequency(100);


    }

    @Override
    public double getAngleRad() {
        BaseStatusSignal.refreshAll(yaw);
        return Units.degreesToRadians(yaw.getValueAsDouble());
    }
}
