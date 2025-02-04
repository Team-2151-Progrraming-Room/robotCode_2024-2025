package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;



// SparkMax imports - these come from REV Robotics

import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;


// our robot constants

import frc.robot.Robot;
import frc.robot.Constants.ArmConstants;
import frc.robot.Utilities;



import java.util.function.DoubleSupplier;



public class ArmSubsystem extends SubsystemBase{
    private SparkPIDController m_armPIDController;
    private final CANSparkMax m_armMotor = new CANSparkMax(ArmConstants.kArmMotor, MotorType.kBrushless);
    private final CANSparkMax m_lockMotor = new CANSparkMax(ArmConstants.kLockMotor, MotorType.kBrushless);
    private final RelativeEncoder m_armEncoder;
    private int positionCheck = 0;


    public ArmSubsystem(){
        m_armMotor.restoreFactoryDefaults();
        m_armMotor.stopMotor();
        m_amMotor.setSmartCurrentLimit(ArmConstants.kArmMotorCurrentLimit);

        m_armPIDController.setP(ArmConstants.kArmPIDControllerP);
        m_armPIDController.setI(ArmConstants.kArmPIDControllerI);
        m_armPIDController.setD(ArmConstants.kArmPIDControllerD);
        m_armPIDController.setFF(ArmConstants.kArmPIDControllerFF);
        m_armPIDController.setIZone(ArmConstants.kArmPIDControllerIZone);
        m_armPIDController.setOutputRange(ArmConstants.kArmPIDControllerOutputMin, ArmConstants.kArmPIDControllerOutputMax);
        m_armPIDController.setReference (ArmConstants.kArmPositionLow, CANSparkMax.ControlType.kPosition);


        m_armPIDController = m_armMotor.getPIDController();
        m_armEncoder = m_armMotor.getEncoder();


    }

    public void setArmPositionLow(){
        m_armPIDController.setReference(ArmConstants.kArmPositionLow, CANSparkBase.ControlType.kPosition);

        positionCheck = ArmConstants.kArmPositionLow;
    }

    public void setArmPositionMid(){
        m_armPIDController.setReference(ArmConstants.kArmPositionMid, CANSparkBase.ControlType.kPosition);
    }

    public void setArmPositionHigh(){
        m_armPIDController.setReference(ArmConstants.kArmPositionHigh, CANSparkBase.ControlType.kPosition);
    }

    public boolean checkArmPosition(){
        if (MathUtil.isNear(positionCheck)){
            return true;
        }
        
        return false;
    }
}
