package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants.*;

public class Arm extends SubsystemBase {
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private final ArmIO io;
    private MechanismLigament2d armMechanism;
    private double setpoint = 0;
    
    public Arm(ArmIO armIO) {
        this.io = armIO;
        SmartDashboard.putData(getName(), this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("PivotArm", inputs);

        armMechanism.setAngle(inputs.angle);

        // Update the PID constants if they have changed
        // if (p.get() != io.getP()) 
        //     io.setP(p.get());
        
        // if (i.get() != io.getI())
        //     io.setI(i.get());
        
        // if (d.get() != io.getD())
        //     io.setD(d.get());
        
        // if (ff.get() != io.getFF())
        //     io.setFF(ff.get());
        
        // Log Inputs
        Logger.getInstance().processInputs("PivotArm", inputs);
    }
    
    public void setVoltage(double motorVolts) {
        // limit the arm if its past the limit
        if (io.getAngle() > ArmIO.PIVOT_ARM_MAX_ANGLE && motorVolts > 0) {
            motorVolts = 0;
        } else if (io.getAngle() < ArmIO.PIVOT_ARM_MIN_ANGLE && motorVolts < 0) {
            motorVolts = 0;
        }
        
        io.setVoltage(motorVolts);
    }

    public void move(double speed) {
        setVoltage(speed * 12);
    }

    public void runPID() {
        io.goToSetpoint(setpoint);
    }

    public void setPID(double setpoint) {
        this.setpoint = setpoint;
    }

    public boolean atSetpoint() {
        return Math.abs(io.getAngle() - setpoint) < ARM_PID_TOLERANCE;
    }

    public Command PIDCommand(double setpoint) {
        return new FunctionalCommand(
            () -> setPID(setpoint), 
            () -> runPID(), 
            (stop) -> move(0), 
            this::atSetpoint, 
            this
        );
    }

    public void setMechanism(MechanismLigament2d mechanism) {
        armMechanism = mechanism;
    }

    public MechanismLigament2d append(MechanismLigament2d mechanism) {
        return armMechanism.append(mechanism);
    }

    public MechanismLigament2d getArmMechanism() {
        return new MechanismLigament2d("Pivot Arm", 2, 0, 5, new Color8Bit(Color.kAqua));
    }

}

