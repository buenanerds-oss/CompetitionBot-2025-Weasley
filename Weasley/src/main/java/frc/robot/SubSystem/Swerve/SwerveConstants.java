package frc.robot.SubSystem.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {

    //dimensions
    public static final double driveWidth = 0;
    public static final double driveLength = 0;

    //turn motor
    public static final double turnGearRatio = 8.75;
    public static final double turnMaxAmps = 5;
    public static final double turnMaxVolts = 1;
    public static final double turnAccuracyToleranceRAD = Units.degreesToRadians(1);

    //drive motor
    public static final double driveGearRatio = 1;
    public static final double driveMaxAmps = 3;
    public static final double driveMaxVolts = 3;
    public static final double driveAccuracyToleranceMPS = 0.1; // meters per second, imperial system's for losers

    //other
    public static final double timeStepToMove = 0.2;
    public static final Pose2d initialPose = new Pose2d();
    public static final double wheelRadiusMeters = Units.inchesToMeters(1.5);
    //Order: FL,FR,BL,BR. for anything numbered accross the modules, assume this order.
    public static final Translation2d[] moduletranslations = {new Translation2d(driveLength/2, driveWidth/2), new Translation2d(driveLength/2, -driveWidth/2),
        new Translation2d(-driveLength/2, driveWidth/2), new Translation2d(-driveLength/2, -driveWidth/2)};
    public static final double mass_KG = 45.3592;
    public static final double MomentOfInertia = 0.00;
    public static final int GyroCANID = 10;


}
