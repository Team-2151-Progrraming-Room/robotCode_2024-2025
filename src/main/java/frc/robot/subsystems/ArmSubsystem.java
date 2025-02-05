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
import frc.robot.Constants.ArmConstants;
import frc.robot.util.*;



import java.util.function.DoubleSupplier;



public class ArmSubsystem extends SubsystemBase{
    private SparkClosedLoopController m_armPIDController;
    SparkMaxConfig config = new SparkMaxConfig();
    private final SparkMax m_armMotor = new SparkMax(ArmConstants.kArmMotor, MotorType.kBrushless);
    private final SparkMax m_lockMotor = new SparkMax(ArmConstants.kLockMotor, MotorType.kBrushless);
    private int positionCheck = 0;


    /*
     * Constructor for arm subsystem- configures pid controller
     */
    public ArmSubsystem(){
        m_armMotor.stopMotor();

        m_armPIDController.setReference (ArmConstants.kArmPositionLow, SparkMax.ControlType.kPosition);

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

        m_armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_armPIDController = m_armMotor.getClosedLoopController();


    }

    /*
     * Methods to set the arm to different positions
     */

    public void setArmPositionLow(){
        m_armPIDController.setReference(ArmConstants.kArmPositionLow, ControlType.kPosition);

        positionCheck = ArmConstants.kArmPositionLow;
    }

    public void setArmPositionMid(){
        m_armPIDController.setReference(ArmConstants.kArmPositionMid, ControlType.kPosition);
    }

    public void setArmPositionHigh(){
        m_armPIDController.setReference(ArmConstants.kArmPositionHigh, ControlType.kPosition);
    }

    //returns the positition of the arm
    public double getPosition(){
        double position = m_armMotor.getEncoder().getPosition();
        return position;
    }

    //checks to see if the arm in close to the position that it should be at
    public boolean checkArmPosition(){
        if (MathUtil.isNear(positionCheck, getPosition(),ArmConstants.kArmPIDControllerOutputRange)){
            return true;
        }
        
        return false;
    }

    /*
     * Commands for different methods
     */

     public Command setArmPositionLowCommand(){
        return runOnce(
            () -> {setArmPositionLow();}
        );
     }

     public Command setArmPositionMidCommand(){
        return runOnce(
            () -> {setArmPositionMid();}
        );
     }

     public Command setArmPositionHighCommand(){
        return runOnce(
            () -> {setArmPositionHigh();}
        );
     }

     public Command checkArmPositionCommand(){
        return runOnce(
            () -> {checkArmPosition();}
        );
     }
}