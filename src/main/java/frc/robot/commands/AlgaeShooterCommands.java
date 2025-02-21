// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.AlgaeSubsystemCTRE;


import frc.robot.Constants.*;

import static edu.wpi.first.units.Units.*;


public class AlgaeShooterCommands extends Command{

AlgaeSubsystemCTRE m_algaeSubsystem;

//Shooting Command (maybe not be needed)

public AlgaeShooterCommands(AlgaeSubsystemCTRE AlgaeSystem){

    m_algaeSubsystem = AlgaeSystem;
    
    addRequirements(AlgaeSystem);
}


public Command getShootCommand(){

    return Commands.sequence(
        
            m_algaeSubsystem.RevMotorsSHOOTCommand(),
            Commands.waitSeconds(AlgaeConstants.ShooterWaitTime),
           
            m_algaeSubsystem.KickMotorONCommand(),
            Commands.waitSeconds(AlgaeConstants.ShooterWaitTime),
            m_algaeSubsystem.allMotorsOFFCommand()
    );
    

        

        




        
    


}

// Dumping Command







// Ground Intake Command






// Processor Deposit Command







}