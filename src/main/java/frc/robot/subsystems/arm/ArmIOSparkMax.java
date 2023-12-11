package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ArmIOSparkMax implements ArmIO{
    
    private CANSparkMax armMotor;
    private SparkMaxPIDController pidController;
    private RelativeEncoder encoder;
    private DutyCycleEncoder absoluteEncoder;
    private double setpoint;

    public ArmIOSparkMax () {
        armMotor = new CANSparkMax(0, CANSparkMax.MotorType.kBrushless);
        pidController = armMotor.getPIDController();
        encoder = armMotor.getEncoder();
        absoluteEncoder = new DutyCycleEncoder(0);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.angle = encoder.getPosition();
        inputs.angleRadsPerSec = encoder.getVelocity();
        inputs.appliedVolts = armMotor.getAppliedOutput() * armMotor.getBusVoltage();
        // inputs.currentAmps = new double[] {pivotMotor.getOutputCurrent()};
        // inputs.tempCelsius = new double[] {pivotMotor.getMotorTemperature()};
    }

    @Override
    public double getAngle() {
        return encoder.getPosition();
    }

    @Override
    public void setVoltage(double motorVolts) {
        armMotor.setVoltage(motorVolts);
    }


 
     /** Go to Setpoint */
     @Override
     public void goToSetpoint(double setpoint) {
         this.setpoint = setpoint;
         pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
     }
 
     @Override
     public void setBrake(boolean brake) {
         armMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
     }
 
     @Override
     public boolean atSetpoint() {
        // return Math.abs(encoder.getPosition() - setpoint) < PivotArm.PIVOT_ARM_PID_TOLERANCE;
        return false;
    }
 
     @Override
     public void setP(double p) {
         pidController.setP(p);
     }
 
     @Override
     public void setI(double i) {
         pidController.setI(i);
     }
 
     @Override
     public void setD(double d) {
         pidController.setD(d);
     }
 
     @Override
     public void setFF(double ff) {
         pidController.setFF(ff);
     }
 
     @Override
     public double getP() {
         return pidController.getP();
     }
 
     @Override
     public double getI() {
         return pidController.getI();
     }
 
     @Override
     public double getD() {
         return pidController.getD();
     }
 
     @Override
     public double getFF() {
         return pidController.getFF();
     }
 

}

