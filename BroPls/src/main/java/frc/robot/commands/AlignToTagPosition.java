package frc.robot.commands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MultiTagLimelight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.LimelightConstants;

public class AlignToTagPosition extends Command {
    private final MultiTagLimelight m_limelight;
    private final SwerveSubsystem m_swerveSubsystem;

    // PID Controllers
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_rotationController;

    // Alignment Parameters
    private static final double X_kP = 1.5;
    private static final double Y_kP = 1.5;
    private static final double ROTATION_kP = 0.5;
    private static final double POSITION_TOLERANCE = 0.05; // 5 cm
    private static final double ROTATION_TOLERANCE = 5.0; // 5 degrees

    // Tracking variables
    private boolean m_alignmentComplete = false;

    public AlignToTagPosition(MultiTagLimelight limelight, SwerveSubsystem swerveSubsystem) {
        m_limelight = limelight;
        m_swerveSubsystem = swerveSubsystem;

        // Initialize PID Controllers
        m_xController = new PIDController(X_kP, 0, 0.1);
        m_yController = new PIDController(Y_kP, 0, 0.1);
        m_rotationController = new PIDController(ROTATION_kP, 0, 0.05);

        // Configure PID Tolerances
        m_xController.setTolerance(POSITION_TOLERANCE);
        m_yController.setTolerance(POSITION_TOLERANCE);
        m_rotationController.setTolerance(ROTATION_TOLERANCE);
        m_rotationController.enableContinuousInput(-180, 180);

        addRequirements(m_limelight, m_swerveSubsystem);
    }

    @Override
    public void initialize() {
        // Clear any previous drive controller overrides
        PPHolonomicDriveController.clearXFeedbackOverride();
        PPHolonomicDriveController.clearYFeedbackOverride();
        PPHolonomicDriveController.clearRotationFeedbackOverride();

        // Reset flags and PID controllers
        m_alignmentComplete = false;
        m_xController.reset();
        m_yController.reset();
        m_rotationController.reset();

        // Validate target visibility
        if (!m_limelight.hasValidTarget()) {
            end(true);
            return;
        }

        // Determine target pose based on current vision data
        Pose2d currentPose = m_swerveSubsystem.getPose();

        // Set PID Setpoints 
        m_xController.setSetpoint(currentPose.getX());
        m_yController.setSetpoint(currentPose.getY());
        m_rotationController.setSetpoint(currentPose.getRotation().getDegrees());
    }

    @Override
    public void execute() {
        if (!m_limelight.hasValidTarget()) {
            // Use Translation2d and rotation for drive method
            m_swerveSubsystem.drive(
                new Translation2d(0, 0), 
                0, 
                false
            );
            return;
        }

        // Calculate PID outputs
        double xSpeed = m_xController.calculate(m_swerveSubsystem.getPose().getX());
        double ySpeed = m_yController.calculate(m_swerveSubsystem.getPose().getY());
        double rotationSpeed = m_rotationController.calculate(
            m_swerveSubsystem.getPose().getRotation().getDegrees()
        );

        // Use Translation2d and rotation for drive method
        m_swerveSubsystem.drive(
            new Translation2d(xSpeed, ySpeed), 
            rotationSpeed, 
            false
        );

        // Update SmartDashboard for debugging
        SmartDashboard.putNumber("Alignment X Error", m_xController.getPositionError());
        SmartDashboard.putNumber("Alignment Y Error", m_yController.getPositionError());
        SmartDashboard.putNumber("Alignment Rotation Error", m_rotationController.getPositionError());
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        m_swerveSubsystem.drive(
            new Translation2d(0, 0), 
            0, 
            false
        );
        m_alignmentComplete = true;

        // Reset Limelight targeting
        m_limelight.resetTarget();
    }

    @Override
    public boolean isFinished() {
        return m_alignmentComplete || 
               (m_xController.atSetpoint() && 
                m_yController.atSetpoint() && 
                m_rotationController.atSetpoint());
    }
}