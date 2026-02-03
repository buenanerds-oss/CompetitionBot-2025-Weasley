package frc.robot;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.AnalogEncoder;

public class RobotMap {
    private static double[] swerveAbsZeroPos = { 
        4.071693,
        2.830042 + Math.PI,
        5.274043,
        1.992770 + Math.PI
    };

    public static SparkMax[] SwerveTurnMotors = {new SparkMax(4, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless),
    new SparkMax(2, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless), 
    new SparkMax(8, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless),
    new SparkMax(6, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless)}; //FL, FR, BL, BR for both Swerve Motors
    public static SparkMax[] SwerveDriveMotors = { new SparkMax(3, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless),
    new SparkMax(1, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless),
    new SparkMax(7, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless),
    new SparkMax(5, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless)};
    public static AnalogEncoder[] swerveAbsEncoders = {//FL, FR, BL, BR
        new AnalogEncoder(0, 2*Math.PI, swerveAbsZeroPos[0]),
        new AnalogEncoder(1, 2*Math.PI, swerveAbsZeroPos[1]),
        new AnalogEncoder(2, 2*Math.PI, swerveAbsZeroPos[2]),
        new AnalogEncoder(3, 2*Math.PI, swerveAbsZeroPos[3])
    };
}
