package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drive;

public class PathPlannerStuff {
    
    public PathPlannerTrajectory trajectory = PathPlanner.loadPath("New Path", new PathConstraints(4.0, 3.0));

    private Drive mDrive;
    private PIDController thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);

    public PathPlannerStuff(Drive drive){
        mDrive = drive;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.reset();
    }

    public PPSwerveControllerCommand getPathFollowCommand(PathPlannerTrajectory traj){
        thetaController.reset();
        PPSwerveControllerCommand command =
            new PPSwerveControllerCommand(traj,
             mDrive::getPose,
             new PIDController(AutoConstants.kPXController, 0, 0),
             new PIDController(AutoConstants.kPXController, 0, 0),
             thetaController,
             mDrive::setModuleStates, 
             mDrive);
        return command;
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory trajectory, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(
                        () -> {
                            if (isFirstPath) {
                                mDrive.resetOdometry(trajectory.getInitialHolonomicPose());
                                // m_swerve.setHeadingOffset(trajectory.getInitialHolonomicPose().getRotation().getDegrees());

                            }
                        }),
                getPathFollowCommand(trajectory));
    }

}
