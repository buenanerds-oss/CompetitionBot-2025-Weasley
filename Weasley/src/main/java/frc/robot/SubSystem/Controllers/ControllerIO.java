package frc.robot.SubSystem.Controllers;

public interface ControllerIO {

    /**
     * positive x = forward
     * @return
     */
    public default double getDriveX() { return 0.0;}

    /**
     * positive Y = left
     * @return
     */
    public default double getDriveY() {return 0.0;}

    /**
     * positive Twist = counterClockwise
     * @return
     */
    public default double getDriveTwist() { return 0.0;}
}
