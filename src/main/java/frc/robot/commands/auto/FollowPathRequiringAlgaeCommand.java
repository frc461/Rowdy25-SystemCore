package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.Event;
import com.pathplanner.lib.events.OneShotTriggerEvent;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.vision.PhotonUtil;

import java.util.*;
import java.util.function.BooleanSupplier;

public class FollowPathRequiringAlgaeCommand extends FollowPathCommand {
    private final Timer timer = new Timer();
    private final Swerve swerve;
    private final PathPlannerPath originalPath;
    private final RobotConfig robotConfig;
    private final BooleanSupplier shouldFlipPath;
    private final BooleanSupplier hasAlgaeTargets = PhotonUtil.Color::hasAlgaeTargets;
    private final List<OneShotTriggerEvent> allInstantEvents = new ArrayList<>();
    private PathPlannerPath path;
    private PathPlannerTrajectory trajectory;
    protected boolean interrupted;

    public FollowPathRequiringAlgaeCommand(PathPlannerPath path, boolean setAssumedPosition, Swerve swerve) {
        /* ah, */ super(
                path,
                swerve.localizer::getStrategyPose,
                () -> swerve.getKinematics().toChassisSpeeds(swerve.getState().ModuleStates),
                (speeds, feedforwards) -> swerve.setControl(new SwerveRequest.ApplyRobotSpeeds()
                        .withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY())
                ),
                new PPHolonomicDriveController(
                        new PIDConstants(
                                Constants.SwerveConstants.PATH_TRANSLATION_CONTROLLER_P,
                                0,
                                0
                        ),
                        new PIDConstants(
                                Constants.SwerveConstants.PATH_ROTATION_CONTROLLER_P,
                                0,
                                0
                        )
                ),
                Constants.AutoConstants.ROBOT_CONFIG,
                () -> Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red,
                swerve
        );

        this.swerve = swerve;
        if (setAssumedPosition) {
            path.getStartingHolonomicPose().ifPresent(this.swerve.localizer::setPoses);
        }
        this.originalPath = path;
        this.robotConfig = Constants.AutoConstants.ROBOT_CONFIG;
        this.shouldFlipPath = () -> Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red;

        this.path = this.originalPath;
        Optional<PathPlannerTrajectory> idealTrajectory = this.path.getIdealTrajectory(this.robotConfig);
        idealTrajectory.ifPresent((traj) -> this.trajectory = traj);

        interrupted = false;
    }

    @Override
    public void initialize() {
        /* ah, */ super.initialize();

        if (this.shouldFlipPath.getAsBoolean() && !this.originalPath.preventFlipping) {
            this.path = this.originalPath.flipPath();
        } else {
            this.path = this.originalPath;
        }

        Pose2d currentPose = swerve.localizer.getStrategyPose();
        ChassisSpeeds currentSpeeds = swerve.getKinematics().toChassisSpeeds(swerve.getState().ModuleStates);
        double linearVel = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        if (this.path.getIdealStartingState() != null) {
            boolean idealVelocity = Math.abs(linearVel - this.path.getIdealStartingState().velocityMPS()) <= (double)0.25F;
            boolean idealRotation = !this.robotConfig.isHolonomic || Math.abs(currentPose.getRotation().minus(this.path.getIdealStartingState().rotation()).getDegrees()) <= (double)30.0F;
            if (idealVelocity && idealRotation) {
                this.trajectory = this.path.getIdealTrajectory(this.robotConfig).orElseThrow();
            } else {
                this.trajectory = this.path.generateTrajectory(currentSpeeds, currentPose.getRotation(), this.robotConfig);
            }
        } else {
            this.trajectory = this.path.generateTrajectory(currentSpeeds, currentPose.getRotation(), this.robotConfig);
        }

        List<Event> allEvents = trajectory.getEvents();
        for (Event event : allEvents) {
            if (event instanceof OneShotTriggerEvent && ((OneShotTriggerEvent) event).getEventName().equals(Constants.AutoConstants.ALGAE_CHECK_MARKER)) {
                allInstantEvents.add((OneShotTriggerEvent) event);
            }
        }

        this.timer.reset();
        this.timer.start();
        interrupted = false;
    }

    @Override
    public void execute() {
        /* ah, */ super.execute();

        double currentTime = this.timer.get();
        if (!allInstantEvents.isEmpty()) {
            if (allInstantEvents.get(0).getTimestampSeconds() <= currentTime) {
                allInstantEvents.remove(0);
                if (!hasAlgaeTargets.getAsBoolean()) {
                    interrupted = true;
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        /* ah, */ super.end(this.interrupted);
        this.interrupted = false;
    }

    @Override
    public boolean isFinished() {
        return this.timer.hasElapsed(this.trajectory.getTotalTimeSeconds()) || interrupted;
    }

    @Override
    public WrapperCommand finallyDo(BooleanConsumer end) {
        return new WrapperCommand(this) {
            @Override
            public void end(boolean interrupted) {
                end.accept(FollowPathRequiringAlgaeCommand.this.interrupted || interrupted);
                /* ah, */ super.end(interrupted);
            }
        };
    }
}
