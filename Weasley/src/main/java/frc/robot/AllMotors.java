package frc.robot;

import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.spark.SparkMax;

public class AllMotors {

    public static SparkMax[] SwerveTurnMotors = {new SparkMax(4, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless),
    new SparkMax(2, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless), 
    new SparkMax(8, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless),
    new SparkMax(6, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless)}; //FL, FR, BL, BR for both Swerve Motors
    public static SparkMax[] SwerveDriveMotors = { new SparkMax(3, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless),
    new SparkMax(1, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless),
    new SparkMax(7, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless),
    new SparkMax(5, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless)};
}
