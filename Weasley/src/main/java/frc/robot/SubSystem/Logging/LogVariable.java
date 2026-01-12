package frc.robot.SubSystem.Logging;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

//marks the variable for logging
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
public @interface LogVariable {

}
