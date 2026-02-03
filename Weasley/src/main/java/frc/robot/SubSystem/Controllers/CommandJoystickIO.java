package frc.robot.SubSystem.Controllers;

import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandJoystickIO implements ControllerIO {
    private enum Axis {
         // Joystick
        JOYSTICK_FORWARD(1), JOYSTICK_SIDE(0), JOYSTICK_ROTATION(5),
        // Throttle
        THROTTLE_MAIN(2), THROTTLE_SLIDER(6), THROTTLE_DIAL_SMALL(3), THROTTLE_DIAL_BIG(4);

        int axis;

        Axis(int axis) {
            this.axis = axis;
        }

        public int getAxis() {
            return axis;
        }

    }
    private enum Buttons{
         // Joystick
        FIRE(2), DOUBLE_TRIGGER_1(1), DOUBLE_TRIGGER_2(15), 
        A(3), B(4), C(5), PINKY_TRIGGER(6),
         DPAD_UP_JOYSTICK(16), DPAD_RIGHT_JOYSTICK(17), DPAD_DOWN_JOYSTICK(18), DPAD_LEFT_JOYSTICK(19),
         T1(9), T2(10), T3(11), T4(12), T5(13), T6(14),
         MODE_UP(24), MODE_MIDDLE(25), MODE_DOWN(26),
        // Throttle
        D(7), E(8), I(30),
         FUNCTION(27), START_STOP(28), RESET(29),
         DPAD_UP_THROTTLE(20), DPAD_RIGHT_THROTTLE(21), DPAD_DOWN_THROTTLE(22), DPAD_LEFT_THROTTLE(23),
         MOUSE_BUTTON(31), PRESS_WHEEL(32);

         int button;

        Buttons(int button) {
            this.button = button;
         }

         public int getButton() {
            return button;
         }

    }

    CommandJoystick joystick;

    public CommandJoystickIO(int port) {
        this.joystick = new CommandJoystick(port);
    }

    @Override
    public double getDriveX() {
        return joystick.getX();
    }

    @Override
    public double getDriveY() {
        return joystick.getY();
    }

    @Override
    public double getDriveTwist() {
        return joystick.getTwist();
    }

    @Override
    public Trigger getPIDIncrease() {
        return joystick.button(Buttons.T1.getButton());
        
    }

    @Override
    public Trigger getPIDDecrease() {
        return joystick.button(Buttons.T2.getButton());
    }

    @Override
    public Trigger getPIDSwitchPositive() {
        return joystick.button(Buttons.T3.getButton());
    }

    @Override
    public Trigger getPIDSwitchNegative() {
        return joystick.button(Buttons.T4.getButton());
    }
    


    
}
