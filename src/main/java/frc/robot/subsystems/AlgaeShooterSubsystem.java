package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;

import static edu.wpi.first.units.Units.*;

import java.io.ObjectInputFilter.Config;

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
//import frc.robot.util - keep if needed for future use;
import frc.robot.Constants.ArmConstants;

public class AlgaeShooterSubsystem extends SubsystemBase {
    

    private SparkClosedLoopController m_AlgaePIDController;
    SparkMaxConfig config = new SparkMaxConfig();
    private final SparkMax m_AlgaeRevMotor = new SparkMax(AlgaeShooterConstants.AlgaeRevMotor, MotorType.kBrushless);
    private final SparkMax m_AlgaeRev2Motor = new SparkMax(AlgaeShooterConstants.AlgaeRev2Motor, MotorType.kBrushless);
    private final SparkMax m_AlgaeKickMotor = new SparkMax(AlgaeShooterConstants.AlgaeKickMotor, MotorType.kBrushless);
    
    
    
     /*
     * Constructor for Algae subsystem- configures pid controller
     */
    public AlgaeShooterSubsystem(){

        // m_AlgaeKickMotor.restoreFactoryDefaults();
        //m_AlgaeRevMotor.restoreFactoryDefaults();
        //m_AlgaeRev2Motor.restoreFactoryDefaults();

/// up above is for safety
        


        // For RevMotor

        m_AlgaePIDController.setReference (AlgaeShooterConstants.AlgaeRevVelocity, SparkMax.ControlType.kVelocity);
        m_AlgaeRevMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); //No this is for safety.
        m_AlgaePIDController = m_AlgaeRevMotor.getClosedLoopController();

        config
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ArmConstants.kArmMotorCurrentLimit);
        config.encoder
        .positionConversionFactor(ArmConstants.kArmEncoderConversionFactor);
        config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ArmConstants.kArmPIDControllerP, ArmConstants.kArmPIDControllerI, ArmConstants.kArmPIDControllerD)
        .outputRange(ArmConstants.kArmPIDControllerOutputMin, ArmConstants.kArmPIDControllerOutputMax);

        //For Rev2 Motor

        m_AlgaePIDController.setReference (AlgaeShooterConstants.AlgaeRev2Velocity, SparkMax.ControlType.kVelocity);
        m_AlgaeRev2Motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);        // this is for safety too
        m_AlgaePIDController = m_AlgaeRev2Motor.getClosedLoopController();


        //For Kick Motor

        m_AlgaePIDController.setReference (AlgaeShooterConstants.AlgaeKickVelocity,ControlType.kVelocity);
        m_AlgaeKickMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // this one too.
        m_AlgaePIDController = m_AlgaeKickMotor.getClosedLoopController();
        
        
    }



    public void AlgaeKickMotorOn(){
        m_AlgaeKickMotor.set(AlgaeKickVelocity.kVelocity);
    }
    public void AlgaeRevMotorOn(){
        m_AlgaeRevMotor.set(AlgaeRevVelocity.kVelocity);
 
    }
    public void AlgaeRev2MotorOn(){
        m_AlgaeRev2Motor.set  (AlgaeRev2Motor.kVelocity);
    }
    public void  AlgaeMotorOff(){
        m_AlgaeKickMotor.stopMotor();
        m_AlgaeRevMotor.stopMotor();
        m_AlgaeRev2Motor.stopMotor();
    }
}






