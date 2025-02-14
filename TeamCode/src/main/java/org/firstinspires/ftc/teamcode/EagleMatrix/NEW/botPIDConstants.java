package org.firstinspires.ftc.teamcode.EagleMatrix.NEW;

import com.acmerobotics.dashboard.config.Config;

@Config
public class botPIDConstants {
    // TODO: Add in other frequently used constant values such as PIDF values and ticks/inch-degrees values
    public static class Arm_Constants {
        // TODO: Find values for Arm_Score_Baskets, Arm_Score_Chambers, Arm_PickUp_Sample
        public static final double Arm_Score_Baskets = 3000;
        public static final double Arm_Score_Chambers = 0;
        public static final double Arm_PickUp_Sample = 0;
        public static final double Arm_Home = 0;
    }

    public static class Lift_Constants {
        // TODO: Find values for Lift_Score_Baskets, Lift_Score_Chambers, Lift_Hang
        public static final double Lift_Score_Baskets = 0;
        public static final double Lift_Score_Chambers = 0;
        public static final double Lift_Hang = 0;
        public static final double Lift_Home = 0;
    }

    public static class Drive_Constants {
        // TODO: Find coordinates for ObservationZone, NetZone, Chambers
        public static final double ObservationZoneX = 0, ObservationZoneY = 0, ObservationZoneHeading = 0;
        public static final double NetZoneX = 0, NetZoneY = 0, NetZoneHeading = 0;
        public static final double ChambersX = 0, ChambersY = 0, ChambersHeading = 0;
    }
}
