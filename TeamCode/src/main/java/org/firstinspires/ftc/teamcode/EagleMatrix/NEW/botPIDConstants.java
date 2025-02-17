package org.firstinspires.ftc.teamcode.EagleMatrix.NEW;

import com.acmerobotics.dashboard.config.Config;

@Config
public class botPIDConstants {
    // TODO: Add in other frequently used constant values such as PIDF values and ticks/inch-degrees values

    @Config
    public static class PIDF_Constants {
        public static final double Arm_p = 0.01, Arm_i = 0, Arm_d = 0, Arm_f = 0;
        public static final double Lift_p = 0.01, Lift_i = 0, Lift_d = 0, Lift_f = 0;
        public static final double Xp = 0.08, Xi = 0, Xd = 0.003, Xf = 0;
        public static final double Yp = 0.08, Yi = 0, Yd = 0.009, Yf = 0;
    }

    public static double Heading_p = 0.02, Heading_i = 0, Heading_d = 0.001, Heading_f = 0;

    @Config
    public static class Ticks2Deg {
        public static final double ArmTicksInDegree = 700 / 180.0; // TODO: FIND CORRECT VALUES FOR ARM, LIFT, DRIVE, or just use it as it
        public static final double LiftTicksInDegree = 700 / 180.0;
        public static final double DriveTicksInDegree = 700 / 180.0;
    }

    @Config
    public static class Arm_Constants {
        public static final double Arm_Score_High_Baskets = -5000;
        public static final double Arm_Score_High_Chambers = -3500;
        public static final double Arm_PickUp_Sample = -300; // check this
        public static final double Arm_Home = -10;
    }
    public static class Lift_Constants {
        public static final double Lift_Score_Baskets = 6000;
        public static final double Lift_Score_Chambers = 1000;
        public static final double Lift_Hang_Grab = -6000;
        public static final double Lift_Home = 0;
    }
    public static class Drive_Constants {
        // TODO: Find coordinates for ObservationZone, NetZone, Chambers
        public static final double ObservationZoneX = 0, ObservationZoneY = 0, ObservationZoneHeading = 0;
        public static final double NetZoneX = 0, NetZoneY = 0, NetZoneHeading = 0;
        public static final double ChambersX = 0, ChambersY = 0, ChambersHeading = 0;
    }
}