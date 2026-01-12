package frc.robot.Util;

public class BasicUtil {

    /**
     * returns true only if the value passed is + || - tolerance of the desired value
     * @param value - the value that you are comparing
     * @param desiredValue - the value you want it to be
     * @param tolerance - the range of that you'll allow it to be true
     * @return - whether or not value is + || - tolerance of desired value
     */
    public static boolean numIsInBallparkOf(double value, double desiredValue, double tolerance) {
        if (value >= desiredValue - tolerance && value <= desiredValue + tolerance) return true;
        else return false;
    }
}
