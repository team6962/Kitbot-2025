// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int LEFT_LEADER_ID = 14;
    public static final int LEFT_FOLLOWER_ID = 13;
    public static final int RIGHT_LEADER_ID = 11;
    public static final int RIGHT_FOLLOWER_ID = 9;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
  }

  public static final class HangConstants {
    public static final int HANG_MOTOR_ID = 12;
    public static final int HANG_MOTOR_CURRENT_LIMIT = 60;
    public static final double HANG_MOTOR_VOLTAGE_COMP = 10;
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }
}
