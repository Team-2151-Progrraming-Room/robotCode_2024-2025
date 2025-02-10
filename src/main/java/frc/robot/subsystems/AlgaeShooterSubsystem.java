package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;



// SparkMax imports - these come from REV Robotics

import com.revrobotics.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.*;

// Our robot constants

import frc.robot.Robot;
import frc.robot.Constants.AlgaeShooterConstants;
//import frc.robot.util;

public class AlgaeShooterSubsystem extends SubsystemBase {
    

    private SparkClosedLoopController m_AlgaePIDController;
    SparkMaxConfig config = new SparkMaxConfig();
    private final SparkMax m_AlgaeRevMotor = new SparkMax(AlgaeShooterConstants.AlgaeRevMotor, MotorType.kBrushless);
    private final SparkMax m_AlgaeKickMotor = new SparkMax(AlgaeShooterConstants.AlgaeKickMotor, MotorType.kBrushless);
    
    
     /*
     * Constructor for Algae subsystem- configures pid controller
     */
    public AlgaeShooterSubsystem(){
        m_AlgaeKickMotor.stopMotor();
        m_AlgaeRevMotor.stopMotor();

        m_AlgaePIDController.setReference (AlgaeShooterConstants.AlgaeRevVelocity, SparkMax.ControlType.kVelocity);

        config.inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(AlgaeShooterConstants.AlgaeRevMotorCurrentLimit);
        config.encoder
        .positionConversionFactor(AlgaeShooterConstants.AlgaeEncoderConversionFactor);
        config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(AlgaeShooterConstants.AlgaePIDControllerP, AlgaeShooterConstants.AlgaePIDControllerI, AlgaeShooterConstants.AlgaePIDControllerD)
        .outputRange(AlgaeShooterConstants.AlgaePIDControllerOutputMin, AlgaeShooterConstants.AlgaePIDControllerOutputMax);

        m_AlgaeRevMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_AlgaePIDController = m_AlgaeRevMotor.getClosedLoopController();

    

        m_AlgaeKickMotor.stopMotor();

        m_AlgaePIDController.setReference (AlgaeShooterConstants.AlgaeKickVelocity,ControlType.kVelocity);

        config
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(AlgaeShooterConstants.AlgaeKickMotorCurrentLimit);
        config.encoder
        .positionConversionFactor(AlgaeShooterConstants.AlgaeEncoderConversionFactor);
        config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(AlgaeShooterConstants.AlgaePIDControllerP, AlgaeShooterConstants.AlgaePIDControllerI, AlgaeShooterConstants.AlgaePIDControllerD)
        .outputRange(AlgaeShooterConstants.AlgaePIDControllerOutputMin, AlgaeShooterConstants.AlgaePIDControllerOutputMax);

        m_AlgaeRevMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_AlgaePIDController = m_AlgaeKickMotor.getClosedLoopController();
        
        public void AlgaeKickMotorOn() {

            m_AlgaeKickMotor.set(AlgaeShooterConstants.AlgaeSpeedBackward)
        }
            
       

    }









}
