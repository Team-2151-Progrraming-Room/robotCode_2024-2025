// Imports
package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;

import static edu.wpi.first.units.Units.*;

import java.io.ObjectInputFilter.Config;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.robot.Robot;
import frc.robot.Constants.AlgaeConstants;

// Main Class
public class AlgaeSubsystemCTRE extends SubsystemBase{
    TalonFXS m_Rev = new TalonFXS(AlgaeConstants.AlgaeRevMotorID);
    TalonFXS m_Rev2  = new TalonFXS(AlgaeConstants.AlgaeRev2MotorID);
    TalonFXS m_Kick = new TalonFXS(AlgaeConstants.AlgaeKickMotorID);

    TalonFXSConfiguration configs = new TalonFXSConfiguration();
    
    Slot0Configs slot0;
    
    // Sub Class
    public AlgaeSubsystemCTRE (){
        slot0 = configs.Slot0;
        slot0.kS = AlgaeConstants.AlgaePIDControllerS;
        slot0.kV = AlgaeConstants.AlgaePIDControllerV;
        slot0.kP = AlgaeConstants.AlgaePIDControllerP;
        slot0.kI = AlgaeConstants.AlgaePIDControllerI;
        slot0.kD = AlgaeConstants.AlgaePIDControllerD;

        m_Rev.getConfigurator().apply(configs);
        m_Rev.setInverted(true);
        m_Rev2.getConfigurator().apply(configs);

        m_request = new VelocityVoltage(AlgaeConstants.AlgaeVoltage).withSlot(0);
    }

    final VelocityVoltage m_request;

    public void algaeIntake(){
        m_Rev.set(AlgaeConstants.algaeIntake);
        m_Rev2.set(AlgaeConstants.algaeIntake);
        
    }

    // For the Motors to turn OFF!
    public void RevMotorsOFF(){
        m_Rev.stopMotor();
        m_Rev2.stopMotor();
    }

    public void KickMotorOFF(){
        m_Kick.stopMotor();
    }

    public void allMotorsOFF(){
        m_Rev.stopMotor();
        m_Rev2.stopMotor();
        m_Kick.stopMotor();
    }

    // For the Motors to turn ON!
    public void RevMotorsSHOOT(){
        m_Rev.setControl(m_request.withVelocity(AlgaeConstants.AlgaeRevVelocity).withFeedForward(AlgaeConstants.AlgaeFeed));
        m_Rev2.setControl(m_request.withVelocity(AlgaeConstants.AlgaeRev2Velocity).withFeedForward(AlgaeConstants.AlgaeFeed));
    }
    
    public void KickMotorON(){
        m_Kick.set(AlgaeConstants.AlgaeKickMotorON);
    }
// Get velocites, put in later.






    //Commands
    
    public Command RevMotorOFFCommand(){
        return runOnce(
            () -> {RevMotorsOFF();}
        );
    }

    public Command KickMotorOFFCommand(){
        return runOnce(
            () -> {KickMotorOFF();}
        );
    }

    public Command allMotorsOFFCommand(){
        return runOnce(
            () -> {allMotorsOFF();}
        );
    }

    public Command RevMotorsSHOOTCommand(){
        return runOnce(
            () -> {RevMotorsSHOOT();}
        );
    }

    public Command KickMotorONCommand(){
        return runOnce(
            () -> {KickMotorON();}
        );
    }
}