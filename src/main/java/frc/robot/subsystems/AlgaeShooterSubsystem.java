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
    

    private SparkClosedLoopController m_AlgaeRevPIDController;
    private SparkClosedLoopController m_AlgaeRev2PIDController;
    private SparkClosedLoopController m_AlgaeKickPIDController;

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
            //m_AlgaeRevPIDController.setReference (AlgaeShooterConstants.AlgaeRev2Velocity, SparkMax.ControlType.kVelocity);
            m_AlgaeRevMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); //No this is for safety.
            m_AlgaeRevPIDController = m_AlgaeRevMotor.getClosedLoopController();

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

        // m_AlgaeRev2PIDController.setReference (AlgaeShooterConstants.AlgaeRev2Velocity, SparkMax.ControlType.kVelocity);
            m_AlgaeRev2Motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);        // this is for safety too
            m_AlgaeRev2PIDController = m_AlgaeRev2Motor.getClosedLoopController();


            //For Kick Motor

        // m_AlgaeKickPIDController.setReference (AlgaeShooterConstants.AlgaeKickVelocity,ControlType.kVelocity);
            m_AlgaeKickMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // this one too.
            m_AlgaeKickPIDController = m_AlgaeKickMotor.getClosedLoopController();
            
            
    }



    public void AlgaeKickMotorOn(){
            m_AlgaeKickPIDController.setReference(AlgaeShooterConstants.AlgaeKickVelocity, SparkMax.ControlType.kVelocity);
    }

    public void AlgaeRevMotorsOn(){
            m_AlgaeRevPIDController.setReference(AlgaeShooterConstants.AlgaeRevVelocity, SparkMax.ControlType.kVelocity); 
            m_AlgaeRev2PIDController.setReference(AlgaeShooterConstants.AlgaeRev2Velocity, SparkMax.ControlType.kVelocity);
    }

    public void  AlgaeMotorsOff(){
            m_AlgaeKickMotor.stopMotor();
            m_AlgaeRevMotor.stopMotor();
            m_AlgaeRev2Motor.stopMotor();
    }
    
/// For getting the Velocities

    public double getRevVelocity(){
        double velocity = m_AlgaeRevMotor.getEncoder().getVelocity();
        return velocity;
    }

    public double getRev2Velocity(){
        double velocity = m_AlgaeRev2Motor.getEncoder().getVelocity();
        return velocity;

    }

// Commands

    public Command AlgaeKickMotorOnCommand(){
        return runOnce(
            () -> {AlgaeKickMotorOn();}
        );
    }

    public Command AlgaeRevMotorsOnCommand(){
        return runOnce(
            () -> {AlgaeRevMotorsOn();}
        );
    }

    public Command AlgaeMotorsStopCommand(){
        return runOnce(
            () -> {AlgaeMotorsOff();}
        );
    }

    public Command getAlgaeRevVelocityCommand(){
        return runOnce(
            () -> {getRevVelocity();}
        );
    }

    public Command GetAlgaeRev2VelocityCommand(){
        return runOnce(
            () -> {getRev2Velocity();}
        );
    }


}






