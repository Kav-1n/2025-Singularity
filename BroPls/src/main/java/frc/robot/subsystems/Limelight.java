// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LimelightHelpers.RawFiducial;

public class Limelight extends SubsystemBase {

    private String m_limelightName;
    private double m_cameraHeight;
    private double m_mountingAngle;
    private double m_goalHeight;

    public enum Align {
        LEFT,
        RIGHT
    }

    public static Align SIDE = Align.RIGHT;

    public Limelight() {
        // Initialize with values from the MegaTag Field-Space Localization Setup
        this.m_limelightName = "limelight";
        this.m_cameraHeight = 6.305;
        this.m_mountingAngle = 26.2;
        this.m_goalHeight = 12;
        
        // Values from your MegaTag setup
        double forward = 0.1;    // LL Forward
        double side = -0.05;     // LL Right (negative means left)
        double up = 0.13;        // LL Up
        double roll = 0.0;       // LL Roll
        double pitch = 26.2;     // LL Pitch
        double yaw = -1.6;       // LL Yaw
        
        // Configure the Limelight camera pose
        LimelightHelpers.setCameraPose_RobotSpace(
            m_limelightName, forward, side, up, roll, pitch, yaw);
    }

    public String getName() {
        return m_limelightName;
    }

    public double getDistanceToGoalInches() {
        return (m_goalHeight - m_cameraHeight)
            / Math.tan(Units.degreesToRadians(m_mountingAngle + getYAngleOffsetDegrees()));
    }

    public double getDistanceToGoalMeters() {
        return Units.inchesToMeters(getDistanceToGoalInches());
    }

    public void setGoalHeight(double goalHeight) {
        this.m_goalHeight = goalHeight;
    }

    public double getGoalHeight() {
        return m_goalHeight;
    }

    // Offset in Degrees
    public double getYAngleOffsetDegrees() {
        return LimelightHelpers.getTY(m_limelightName);
    }

    public double getXAngleOffsetDegrees() {
        return LimelightHelpers.getTX(m_limelightName);
    }

    public double getXOffsetRadians() {
        return Units.degreesToRadians(getXAngleOffsetDegrees());
    }

    public boolean isTargetVisible() {
        return LimelightHelpers.getTV(m_limelightName);
    }

    public RawFiducial[] getRawFiducials() {
        return LimelightHelpers.getRawFiducials(m_limelightName);
    }

    public RawFiducial getRawFiducial(double id) {
        for (RawFiducial fiducial : getRawFiducials()) {
            if (fiducial.id == id) {
                return fiducial;
            }
        }
        return null;
    }

    public boolean hasRawFiducial(double id) {
        for (RawFiducial fiducial : getRawFiducials()) {
            if (fiducial.id == id) {
                return true;
            }
        }
        return false;
    }

    public double getRawFiducialDistToCamera(RawFiducial rawFiducial) {
        return rawFiducial.distToCamera;
    }

    public double getRawFiducialTX(RawFiducial rawFiducial) {
        return rawFiducial.txnc;
    }

    public double getRawFiducialTY(RawFiducial rawFiducial) {
        return rawFiducial.tync;
    }

    // Pipeline control
    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(m_limelightName, pipeline);
    }

    public int getTargetID() {
        return (int) LimelightHelpers.getFiducialID(m_limelightName);
    }

    public static void setSIDE(Align side) {
        Limelight.SIDE = side;
    }

    public Pose2d getBotPose2d() {
        return LimelightHelpers.getBotPose2d_wpiBlue(m_limelightName);
    }

    public Pose3d getBotPose3d() {
        return LimelightHelpers.getBotPose3d_wpiBlue(m_limelightName);
    }

    public Pose2d getBotPose2d_TargetSpace() {
        return LimelightHelpers.getBotPose3d_TargetSpace(m_limelightName).toPose2d();
    }

    public Pose3d getBotPose3d_TargetSpace() {
        return LimelightHelpers.getBotPose3d_TargetSpace(m_limelightName);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("State Machine/Align", Limelight.SIDE.toString());
        SmartDashboard.putNumber(m_limelightName + "/X offset", getXAngleOffsetDegrees());
        SmartDashboard.putNumber(m_limelightName + "/Y offset", getYAngleOffsetDegrees());
        SmartDashboard.putNumber(m_limelightName + "/distance to goal", getDistanceToGoalMeters());
        SmartDashboard.putBoolean(m_limelightName + "/Target Visible", isTargetVisible());
        SmartDashboard.putNumber(m_limelightName + "/Target ID", getTargetID());
    }
}