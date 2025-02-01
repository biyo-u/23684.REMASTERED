package org.firstinspires.ftc.teamcode.EMLite;

public class Air_Traffic_Control {
    // manages the targests for EMLite
    public static class driveToNetZone {
        // WE MUST START CLOSER TO NET ZONE!!!
        public static final double x = 0;
        public static final double y = 20;
        public static final double heading = 0;
    }

    public static class inbetween_values {
        // FOR DURING THE DRIVING
        public static final double x = 5;
        public static final double y = 0;
        public static final double heading = 0;
    }

    public static class driveToObservationZone {
        // NET ZONE TO OBSERVATION ZONE!!!
        public static final double x = 0;
        public static final double y = -60;
        public static final double heading = 180;
    }
}
