package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightConstants;

public class MultiTagLimelight extends SubsystemBase {
    private final NetworkTable m_limelight;
    private final String m_limelightName;

    // Target tracking
    private int m_currentTagId = -1;
    private boolean m_isLeftSide = false;
    private int m_currentLevel = 2; // Default to L2

    // Camera positioning
    private static final double CAMERA_X_OFFSET = Units.inchesToMeters(-5); // 5 inches left of center
    private static final double CAMERA_Y_OFFSET = Units.inchesToMeters(-4); // 4 inches back of center
    private static final double CAMERA_Z_OFFSET = Units.inchesToMeters(24); // 24 inches off floor
    private static final double CAMERA_PITCH = Math.toRadians(-15); // 15 degrees downward

    public MultiTagLimelight() {
        m_limelightName = "limelight"; // Update with your specific Limelight name
        m_limelight = NetworkTableInstance.getDefault().getTable(m_limelightName);
        
        // Set camera pose relative to robot
        LimelightHelpers.setCameraPose_RobotSpace(
            m_limelightName, 
            CAMERA_Y_OFFSET, // Forward/Back 
            CAMERA_X_OFFSET, // Left/Right
            CAMERA_Z_OFFSET, // Up/Down
            0, // Roll
            Units.radiansToDegrees(CAMERA_PITCH), // Pitch 
            0  // Yaw
        );
    }

    @Override
    public void periodic() {
        updateSmartDashboard();
    }

    // Set current target 
    public void setTarget(int tagId, boolean isLeftSide, int level) {
        m_currentTagId = tagId;
        m_isLeftSide = isLeftSide;
        m_currentLevel = level;
    }

    // Get current Limelight results
    public LimelightResults getLimelightResults() {
        return LimelightHelpers.getLatestResults(m_limelightName);
    }

    // Check if a valid target is visible
    public boolean hasValidTarget() {
        return LimelightHelpers.getTV(m_limelightName);
    }

    // Get robot pose from Limelight
    public Pose2d getRobotPose() {
        return LimelightHelpers.getBotPose2d_wpiBlue(m_limelightName);
    }

    // Get specific target information
    public double getTX() {
        return LimelightHelpers.getTX(m_limelightName);
    }

    public double getTY() {
        return LimelightHelpers.getTY(m_limelightName);
    }

    public double getTA() {
        return LimelightHelpers.getTA(m_limelightName);
    }

    public int getTargetID() {
        return (int) LimelightHelpers.getFiducialID(m_limelightName);
    }

    // Update SmartDashboard with Limelight information
    private void updateSmartDashboard() {
        SmartDashboard.putBoolean("Limelight Valid Target", hasValidTarget());
        SmartDashboard.putNumber("Limelight TX", getTX());
        SmartDashboard.putNumber("Limelight TY", getTY());
        SmartDashboard.putNumber("Limelight TA", getTA());
        SmartDashboard.putNumber("Limelight Target ID", getTargetID());
        SmartDashboard.putNumber("Current Tag", m_currentTagId);
        SmartDashboard.putBoolean("Current Side", m_isLeftSide);
        SmartDashboard.putNumber("Current Level", m_currentLevel);
    }

    // Getters for current state
    public int getCurrentTagId() {
        return m_currentTagId;
    }

    public boolean isLeftSide() {
        return m_isLeftSide;
    }

    public int getCurrentLevel() {
        return m_currentLevel;
    }

    // Reset current target
    public void resetTarget() {
        m_currentTagId = -1;
        m_isLeftSide = false;
        m_currentLevel = 2;
    }
}

