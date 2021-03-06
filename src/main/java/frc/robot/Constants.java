// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveTrainConstants {
        public static final int motorLLeadID = 1;
        public static final int motorLFollow1ID = 2;
        public static final int motorLFollow2ID = 3;
        public static final int motorRLeadID = 4;
        public static final int motorRFollow1ID = 5;
        public static final int motorRFollow2ID = 6;
    }

    public static final class OIConstants {
        public static final int contPort = 0; // port of xbox controller connected
        public static final int driverContPort = 1; 
        public static final double xboxDeadzone = .125;
    }

    public static final class AutoConstants {
        //Conversions for the Falcons
        private static final double pi = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679;
        //public static final double ticksToMeters = ( ((15.0/85.0)*(30.0/40.0)) / 1.0 ) * ( (pi * .1524) / 1.0 ); 
        public static final double ticksToMeters = ( ((15.0/85.0)*(30.0/40.0)) / 1.0 ) * ( (pi * .1524) / 1.0 );
        public static final boolean kGyroReversed = true; //Characterization says this isn't necessary but it seems to perform better....
    }
}
