package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public double angle = 0;
        public double angleRadsPerSec = 0;
        public double appliedVolts = 0.0;
    }

    double PIVOT_ARM_MAX_ANGLE = 0;
    double PIVOT_ARM_MIN_ANGLE = 0;

        /** Updates the set of loggable inputs. */
    public default void updateInputs(ArmIOInputs inputs) {

    } 

      /** Returns the current distance measurement. */
    public default double getAngle() {
        return 0.0;
    }

    /** Run open loop at the specified voltage. */
    public default void setVoltage(double motorVolts) {
    }

    public default void setPIDConstants(double p, double i, double d, double ff) {
    }

    /** Go to Setpoint */
    public default void goToSetpoint(double setpoint) {
    }

    public default void setBrake(boolean brake) {
    }

    public default boolean atSetpoint() {
        return false;
    }

    public default void setP(double p) {}
    
    public default void setI(double i) {}

    public default void setD(double d) {}

    public default void setFF(double ff) {}

    public default double getP() { return 0.0; }

    public default double getI() { return 0.0; }

    public default double getD() { return 0.0; }

    public default double getFF() { return 0.0; }
}
