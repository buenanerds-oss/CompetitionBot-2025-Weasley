package frc.robot;

import org.photonvision.PhotonCamera;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class RobotMap {
    private static double[] swerveAbsZeroPos = { 
        // if experiencing the Desired states going opposite to eachother @ certaint points, add PI to zero that has the issue
        5.573,//2.830042 + Math.PI,//4.071693,
        1.984,//4.071693,//2.830042 + Math.PI,
        3.825,//5.274043,
        1.239//1.992770 + Math.PI
    };

    public static SparkMax[] SwerveTurnMotors = {new SparkMax(2, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless),
    new SparkMax(4, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless), 
    new SparkMax(6, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless),
    new SparkMax(8, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless)}; //FL, FR, BL, BR for both Swerve Motors
    public static SparkMax[] SwerveDriveMotors = { new SparkMax(1, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless),
    new SparkMax(3, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless),
    new SparkMax(5, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless),
    new SparkMax(7, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless)};
    public static AnalogEncoder[] swerveAbsEncoders = {//FL, FR, BL, BR
        new AnalogEncoder(0, 2*Math.PI, swerveAbsZeroPos[0]),
        new AnalogEncoder(1, 2*Math.PI, swerveAbsZeroPos[1]),
        new AnalogEncoder(2, 2*Math.PI, swerveAbsZeroPos[2]),
        new AnalogEncoder(3, 2*Math.PI, swerveAbsZeroPos[3])
    };
    public static int pidgeon2CANID = 9;

    public static SparkMax hopperMotor = new SparkMax(12, MotorType.kBrushless);
    public static SparkMax shooterMotor = new SparkMax(11,  MotorType.kBrushless);
    public static SparkMax climbMotor = new SparkMax(10,  MotorType.kBrushless);

    public static PhotonCamera c270 = new PhotonCamera("C270");
    public static PhotonCamera nexigo = new PhotonCamera("NEXIGO");
    public static Transform3d robotToCameras[] = {
        new Transform3d(0, 0, 0, new Rotation3d(0,0,0)),
        new Transform3d(0, 0, 0, new Rotation3d(0,0,0))
    };
    
}
