package frc.robot.subsystems;
import java.time.Period;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Motor extends SubsystemBase {

    SparkMax cim;
    SparkMaxConfig config = new SparkMaxConfig();
    ProfiledPIDController pid;
    DutyCycleEncoder encoder;
    double targetPosition;
    
    public Motor() {
        cim = new SparkMax(0, MotorType.kBrushless);
        config.inverted(true).idleMode(IdleMode.kBrake);
        pid = new ProfiledPIDController(0, 0, 0, new Constraints(0.5, 1));
        encoder = new DutyCycleEncoder(0);
    }

    public double getPosition() {
        return encoder.get();
    }

    public void setVolts(double volts) {
        cim.setVoltage(volts);
    }

    public void setTargetPosition(double position){
        targetPosition = position;
        pid.setGoal(targetPosition);
    }

    public void usePIDOutput() {
        cim.setVoltage(pid.calculate(targetPosition));
    }

    public void periodic() {
        SmartDashboard.putNumber("Position", getPosition());
    }
}