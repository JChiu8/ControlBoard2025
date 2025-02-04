package frc.robot.subsystems;
import java.time.Period;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Motor extends SubsystemBase {

    SparkMax cim;
    SparkMaxConfig config = new SparkMaxConfig();
    ProfiledPIDController pid;
    // PIDController pid;
    DutyCycleEncoder encoder;
    double targetPosition;
    
    public Motor() {
        cim = new SparkMax(1, MotorType.kBrushed);
        config.inverted(false).idleMode(IdleMode.kBrake);
        
        pid = new ProfiledPIDController(0.13, 0.0000, 0.0000105, new Constraints(720.0, 2*720.0));
        pid.enableContinuousInput(0.0, 360.0);
        encoder = new DutyCycleEncoder(0, 1.0, .0595);
        targetPosition = 90.0;
        pid.setGoal(targetPosition);
        pid.setIZone(.5);
        // encoder = new DutyCycleEncoder(0);
        
    }

    public double getEncoderPosition() {
        return encoder.get() * 360.0 - 9.0;
    }

    public void setVolts(double volts) {
        cim.setVoltage(volts);
        pid.getSetpoint().position = getEncoderPosition();
    }

    public void setTargetPosition(double position){
        targetPosition = position;
        pid.setGoal(targetPosition);
    }

    public void usePIDOutput() {
        cim.setVoltage(-pid.calculate(getEncoderPosition()));
    }

    public void periodic() {
        SmartDashboard.putNumber("Position", getEncoderPosition());
        SmartDashboard.putNumber("setpoint", pid.getSetpoint().position);
        SmartDashboard.putNumber("goal", pid.getGoal().position);
        SmartDashboard.updateValues();

    }
}