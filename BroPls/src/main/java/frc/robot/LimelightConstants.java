package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public final class LimelightConstants {
    // Limelight Pipeline Indices
    public static final int kAprilTagPipeline = 0;
    public static final int kRetroreflectivePipeline = 1;

    // Camera Positioning
    public static final double kCameraHeight = Units.inchesToMeters(24);
    public static final double kCameraPitch = Units.degreesToRadians(-15);
    
    // Scoring Position Constants
    public static class ReefPositions {
        // Define scoring positions for different levels
        public static final class L2 {
            public static final Pose3d RIGHT = new Pose3d(
                0.8, // X distance (meters)
                0.3, // Y distance (meters)
                0.0, // Z height (meters)
                new Rotation3d(0, 0, Math.toRadians(6.46)) // Rotation to face tag
            );

            public static final Pose3d LEFT = new Pose3d(
                0.8, // X distance (meters)
                -0.3, // Y distance (meters)
                0.0, // Z height (meters)
                new Rotation3d(0, 0, Math.toRadians(22.66)) // Rotation to face tag
            );
        }

        // Add similar classes for L1, L3, L4
        
        // Method to retrieve specific position
        public static Pose3d getPosition(int level, boolean isLeft) {
            switch (level) {
                case 2:
                    return isLeft ? L2.LEFT : L2.RIGHT;
                // Add cases for other levels
                default:
                    throw new IllegalArgumentException("Invalid level: " + level);
            }
        }
    }

    // Method to get tag ID based on field position
    public static int getTagIdForPosition(int horizontalPosition) {
        // Implement your specific tag mapping logic
        switch (horizontalPosition) {
            case 0: return 3;  // Left side
            case 1: return 2;  // Middle
            case 2: return 4;  // Right side
            default: return 2; // Default to middle
        }
    }
}

