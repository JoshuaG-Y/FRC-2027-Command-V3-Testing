// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.apriltag.AprilTagFields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.Alliance;

/** Add your docs here. */
public class FieldConstants {
    public static AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); // Use This when Field Gets Release

    public static class AllianceZones {
        public static final double RedToNeutralX = 1.0; // Replace these with the actual values
        public static final double NeutralToBlueX = 2.0; // Replace these with the actual values
    }

    public static boolean inFieldBounds(Pose2d pose) {
        return pose.getX() >= 0 && pose.getX() <= FieldConstants.layout.getFieldLength() && pose.getY() >= 0 && pose.getY() <= FieldConstants.layout.getFieldWidth();
    }

    public static boolean inAllianceZone(Pose2d pose) {
        boolean red = MatchState.getAlliance().orElse(Alliance.RED) == Alliance.RED;
        if (red) {
            return pose.getX() <= AllianceZones.RedToNeutralX;
        } else {
            return pose.getX() >= AllianceZones.NeutralToBlueX;
        }
    }
}
