package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.ClimbLockSubsystem;



public class ClimbLockCommand extends Command {

    ClimbLockSubsystem        m_climbLockSubsystem;



    public ClimbLockCommand(ClimbLockSubsystem climbLockSub) {

        System.out.println("Instantiating ClimbLockCommand ----------------------------------------------");
        m_climbLockSubsystem = climbLockSub;

        addRequirements(climbLockSub);
    }



    public Command getClimbLockCommand() {

        System.out.println("getClimbLockCommand invoked ----------------------------------------------");

        return Commands.sequence(

        // right now we just invoke the command to close the climb locks
        //
        // we might want to include some pre and post-lock LED states - there are some comments about where that would typically go
        // typically those would all run in sequence

            Commands.print("Starting ClimbLock Command +++++++++++++++++++++++++++++++++++++++++++++++"),

            // pre-lock LED sequence could go here
            //
            // m_ledSubsystem.preLockLedSequence(),

            m_climbLockSubsystem.climbLockSecureCageCommand()       // add a trailing "," to this line if we add commands after it

            // post-lock LED sequence could go here
            //
            // m_ledSubsystem.postLockLedSequence()

        );
    }
}
