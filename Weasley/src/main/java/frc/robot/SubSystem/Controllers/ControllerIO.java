package frc.robot.SubSystem.Controllers;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControllerIO {

    /**
     * positive x = forward
     * @return
     */
    public default double getDriveX() {return 0.0;}

    /**
     * positive Y = left
     * @return
     */
    public default double getDriveY() {return 0.0;}

    /**
     * positive Twist = counterClockwise
     * @return
     */
    public default double getDriveTwist() {return 0.0;}

    /**
     * 
     * @return 1.00 if increase, 0.00 if nothing, -1.00 if decrease
     */
   public default double getPIDPChange() {return 0.0;}

   public default Trigger getPIDIncrease() { return new Trigger(() -> false);}
   public default Trigger getPIDDecrease() { return new Trigger(() -> false);}

     /**
     * 
     * @return 1.00 if increase, 0.00 if nothing, -1.00 if decrease
     */
   public default double getPIDIAxis() {
    return 0.0;
    }

   public default Trigger getPIDSwitchPositive() { return new Trigger(() -> false);}
   public default Trigger getPIDSwitchNegative() { return new Trigger(() -> false);}

     /**
     * 
     * @return 1.00 if increase, 0.00 if nothing, -1.00 if decrease
     */
   public default double getPIDDChange() {
    return 0.0;
    }

   
   }
