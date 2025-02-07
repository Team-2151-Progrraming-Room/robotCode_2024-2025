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

private final SparkMax m_Motor1 = new SparkMax(CoralConstants.kCoralMotor1, MotorType.kBrushless);
private final SparkMax m_Motor2 = new SparkMax(CoralConstants.kCoralMotor2, MotorType.kBrushless);

public class CoralSubsystem extends SubsystemBase{

    //motor1 pid
    public CoralSubsystem(){
        m_CoralMotor1.stopMotor();

        m_MotorPIDController.setReference (CoralConstants.kArmPositionLow, SparkMax.ControlType.kPosition);

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
        m_coralPIDController = m_coralMotor.getClosedLoopController();


    }

    //intake Motor1
    public void intakeCoralMotor1(){
        m_coralPIDController.setReference(CoralConstants.kCoralMotor1IntakeSpeed, ControlType.kPosition);
    }

    //intake Motor2
    public void intakeCoralMotor2(){
        m_coralPIDController.setReference(CoralConstants.kCoralMotor2IntakeSpeed, ControlType.kPosition);
    }

    //output Motor1
    public void outputCoralMotor1(){
        m_coralPIDController.setReference(CoralConstants.kCoralMotor1OutputSpeed, ControlType.kPosition);
    }

    //output Motor2
    public void outputCoralMotor2(){
        m_coralPIDController.setReference(CoralConstants.kCoralMotor2OutputSpeed, ControlType.kPosition);
    }
   
    /*Command methods for intake and output */
    public Command intakeCoralCommand(){
        return runOnce(
            () -> {intakeCoral();}
        );
     }

}

