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
import frc.robot.util;

public class AlgaeShooterSubsystem extends SubsystemBase {
    
    private SparkClosedLoopController m_AlgaePIDController;
    private SparkClosedLoopController m_AlgaePIDController;
    SparkMaxConfig config = new SparkMaxConfig();
    private final SparkMax m_AlgaeRevMotor = new SparkMax(AlgaeShooterConstantsants.AlgaeRevMotor, MotorType.kBrushless);
    private final SparkMax m_AlgaeKickMotor = new SparkMax(AlgaeShooterConstants.AlgaeKickMotor, MotorType.kBrushless);
    
    
     /*
     * Constructor for Algae subsystem- configures pid controller
     */
    public AlgaeShooterSubsystem(){
        m_AlgaeRevMotor.stopMotor();

        m_AlgaePIDController.setReference (AlgaeShooterConstants.kArmPositionLow, SparkMax.ControlType.kPosition);

        config
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(AlgaeShooterConstants.AlgaeRevMotorCurrentLimit);
        config.encoder
        .positionConversionFactor(AlgaeShooterConstants.AlgaeEncoderConversionFactor);
        config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(AlgaeShooterConstants.AlgaePIDControllerP, AlgaeConstants.AlgaePIDControllerI, AlgaeConstants.AlgaePIDControllerD)
        .outputRange(AlgaeConstants.AlgaePIDControllerOutputMin, AlgaeConstants.AlgaePIDControllerOutputMax);

        m_AlgaeRevMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_AlgaePIDController = m_AlgaeRevMotor.getClosedLoopController();

    }













}
