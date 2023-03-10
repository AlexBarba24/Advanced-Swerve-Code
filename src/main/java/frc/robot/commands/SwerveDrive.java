package frc.robot.commands;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.ModifyAxis;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class SwerveDrive extends CommandBase{
    //Initialize Variables
    DrivetrainSubsystem m_driveSubsystem;
    DoubleSupplier m_translationXSupplier;
    DoubleSupplier m_translationYSupplier;
    DoubleSupplier m_rotationSupplier;
    Boolean m_precisionMode;
    //Creates controller objects
    private final XboxController m_driverController = new XboxController(0);

    
    //Constructor for SwerveDrive
    /**
     * 
     * @param driveSubsystem drive Subsystem
     * @param translationXSupplier X Value
     * @param translationYSupplier Y value
     * @param rotationSupplier Rotation value
     */
    public SwerveDrive(DrivetrainSubsystem driveSubsystem, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        //sets local variable to member variables
        m_driveSubsystem = driveSubsystem;
        m_translationXSupplier = translationXSupplier;
        m_translationYSupplier = translationYSupplier;
        m_rotationSupplier = rotationSupplier;
 
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute() {
        //modifys axis of controlls to ease control
        ModifyAxis m_axisY = new ModifyAxis(m_translationXSupplier.getAsDouble(), 2);
        ModifyAxis m_axisX = new ModifyAxis(m_translationYSupplier.getAsDouble(), 2);
        ModifyAxis m_axisRot = new ModifyAxis(m_rotationSupplier.getAsDouble(), 2);

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        //drives robot at 50% speed
        if(m_driverController.getRightBumper()){
            m_driveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    m_axisX.m_modifiedValue* 0.5,
                    m_axisY.m_modifiedValue* 0.5,
                    m_axisRot.m_modifiedValue * 0.5,
                    m_driveSubsystem.getGyroscopeRotation()

                )
            );  
        }
        //Drives robot at 25% speed
        else if(m_driverController.getLeftBumper()){
            m_driveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    m_axisX.m_modifiedValue* 0.25,
                    m_axisY.m_modifiedValue* 0.25,
                    m_axisRot.m_modifiedValue * 0.25,
                    m_driveSubsystem.getGyroscopeRotation()
                )
            );  
        }
        //Drive robot at full speed
        else{
            m_driveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    m_axisX.m_modifiedValue, //0.25 normally
                    m_axisY.m_modifiedValue, //""
                    m_axisRot.m_modifiedValue, //0.03 normally
                    m_driveSubsystem.getGyroscopeRotation()
                )
            ); 
        }

    }

    @Override
    public void end(boolean interrupted) {
        //Stops drive base if command ends 
        m_driveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
