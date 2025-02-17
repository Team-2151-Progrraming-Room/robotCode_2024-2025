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

// our robot constants

import frc.robot.Robot;
import frc.robot.Constants.CoralConstants;
import frc.robot.util.*;



import java.util.function.DoubleSupplier;



public class CoralSubsystem extends SubsystemBase{

    private final SparkMax m_Motor1 = new SparkMax(CoralConstants.kCoralMotor1, MotorType.kBrushless);
    private final SparkMax m_Motor2 = new SparkMax(CoralConstants.kCoralMotor2, MotorType.kBrushless);
    private SparkClosedLoopController m_Coral1PIDController;
    private SparkClosedLoopController m_Coral2PIDController;
   
    //motor pid
    public CoralSubsystem(){
        m_CoralMotor1.stopMotor();
        m_CoralMotor2.stopMotor();

        config
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(CoralConstants.kCoralMotorCurrentLimit);
        config.encoder
        .positionConversionFactor(CoralConstants.kCoralEncoderConversionFactor);
        config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(CoralConstants.kCoralPIDControllerP, CoralConstants.kCoralPIDControllerI, CoralConstants.kCoralPIDControllerD)
        .outputRange(CoralConstants.kCoralPIDControllerOutputMin, CoralConstants.kCoralPIDControllerOutputMax);

        m_coralMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_Coral1PIDController = m_CoralMotor1.getClosedLoopController();
        m_Coral2PIDController = m_CoralMotor2.getClosedLoopController();

    }

    //encoders
    public double getVelocity1(){
        double position = m_CoralMotor1.getEncoder().getVelocity1();
        return position;
    }
    public double getVelocity2(){
        double position = m_CoralMotor2.getEncoder().getVelocity2();
        return position;
    }

    //intake 
    public void intakeCoralMotor(){
        m_Coral1PIDController.setReference(CoralConstants.kCoralMotor1IntakeSpeed, ControlType.kPosition);
        m_Coral2PIDController.setReference(CoralConstants.kCoralMotor2IntakeSpeed, ControlType.kPosition);
    }

    //output 
    public void outputCoralMotor(){
        m_Coral1PIDController.setReference(CoralConstants.kCoralMotor1OutputSpeed, ControlType.kPosition);
        m_Coral2PIDController.setReference(CoralConstants.kCoralMotor2OutputSpeed, ControlType.kPosition);
    }
   
    /*Command methods for intake and output */
    public Command intakeCoralCommand(){
        return runOnce(
            () -> {intakeCoral();}
        );
    }

     public Command outputCoralCommand(){
        return runOnce(
            () -> {outputCoral();}
        );
    }
    
    //returns velocity value of motors
    public Command getVelocity1Command(){
        return runOnce(
            () -> {getVelocity1();}
        );
    }
    public Command getVelocity2Command(){
        return runOnce(
            () -> {getVelocity2();}
        );
    }

    
}