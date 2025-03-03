//package org.firstinspires.ftc.teamcode.Old;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.arcrobotics.ftclib.command.Command;
//import com.arcrobotics.ftclib.command.CommandBase;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.Subsystems.Drive;
//
//
//
//@Disabled
//@Autonomous(name = "AutoChamberTest")
//public class TODELETEARCHIVEDAutoChamberTest extends OpMode {
//    Drive drive;
//    VoltageSensor battery;
//    ElapsedTime runtime = new ElapsedTime();
//
//
//    public enum Alliance {RED, BLUE}
//
//    /**
//     * @noinspection unused
//     */
//    public Alliance getAlliance() {
//        return Alliance.RED;
//    }
//
//    @Override
//    public void init() {
//        CommandScheduler.getInstance().reset();
//
//        drive = new Drive(hardwareMap);
//        battery = hardwareMap.voltageSensor.get("Control Hub");
//
//        CommandScheduler.getInstance().registerSubsystem(drive);
//
//        drive.reset();
//    }
//
//    @Override
//    public void init_loop() {
//        drive.readSensors();
//    }
//
//    public Command doNothing(long timeout) {
//        return new CommandBase() {
//        }.withTimeout(timeout);
//    }
//
//    @Override
//    public void start() {
//
//        drive.setPosition(new Pose2D(DistanceUnit.INCH, 32.25, -62, AngleUnit.DEGREES, 0));
//        runtime.reset();
//
//        double score_y = -28.3;
//        double pickup_x = 22;
//        double pickup_y = -61.4;
//
//        CommandScheduler.getInstance().schedule(
//                new SequentialCommandGroup(
//
//                        // score our preload
//                        new ParallelCommandGroup(
//                                //arm.highChamber(),
//                                drive.moveTo(-3.1, score_y, 0).withTimeout(2000)
//                        ),
//                        //arm.highChamberScore(true),
//                        //arm.highChamberScore(false),
//
//                        // get first spike-mark sample
//                        new SequentialCommandGroup(
//                                drive.moveTo(29.9, -46, -45).withTimeout(2400),
/// /                                drive.turnQuickly(-45).withTimeout(2400),
//                                new SequentialCommandGroup(
//                                        doNothing(500)
//                                        //arm.moveTo(40,200,0,0.2,Arm.CLAW_OPEN)
//                                )
//                        ),
//                        //arm.moveTo(0,1800,0,0.2,Arm.CLAW_OPEN),
//                        //arm.moveTo(0,1800,0,0.2,Arm.CLAW_CLOSED),
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                //arm.moveTo(0,1800,0.5,0.45,Arm.CLAW_CLOSED),
//                                        drive.moveTo(30.3, -43.3, -135).withTimeout(1300)
////                                drive.turnQuickly(-135).withTimeout(1300)
//                                )
//                        ),
//                        //drop off in human player zone
//                        //arm.moveTo(0,1800,0.5,0.45,Arm.CLAW_OPEN),
//
//                        //go back to second spike mark
//                        // (the "just turn" timeouts are only 1500 because it "should' be fast)
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        //arm.moveTo(0,1800,0,0.2,Arm.CLAW_OPEN),
//                                        drive.moveTo(40.55, -46, -45).withTimeout(1800)
////                                        drive.turnQuickly(-45).withTimeout(1800)
//                                )
//                        ),
//                        //grab spike mark
//                        //arm.moveTo(0,1800,0,0.2,Arm.CLAW_CLOSED),
//                        //drop in human zone
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        //arm.moveTo(0,1800,0.5,0.45,Arm.CLAW_CLOSED),
//                                        drive.moveTo(30.3, -43.3, -135).withTimeout(1300)
////                                        drive.turnQuickly(-135).withTimeout(1300)
//                                )
//                        ),
//                        //arm.moveTo(0,1800,0.5,Arm.PALM_MIDDLE,Arm.CLAW_OPEN),
//                        //arm.moveTo(0,500,0.5,Arm.PALM_MIDDLE,Arm.CLAW_OPEN),
//
//                        // grab completed specimen from human #1
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        doNothing(500),
//                                        //arm.moveTo(0,10,0.5,Arm.PALM_MIDDLE,Arm.CLAW_OPEN)
//                                        new SequentialCommandGroup(
//                                                drive.moveTo(pickup_x, pickup_y, -90).withTimeout(1800)
////                                                drive.turnQuickly(-90).withTimeout(1800)
//                                        )
//                                )
//                        ),
//                        //arm.moveTo(0, 1750, 0.15, Arm.PALM_MIDDLE, Arm.CLAW_OPEN),
//                        //arm.moveTo(0, 1750, 0.15, Arm.PALM_MIDDLE, Arm.CLAW_CLOSED),
//
//                        // score it
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        //arm.highChamber(),
//                                        drive.moveTo(0.0, score_y, 0).withTimeout(1800)
////                                        drive.turnQuickly(0).withTimeo, ut(1800)
//                                )
//                        ),
//                        //arm.highChamberScore(true),
//                        //arm.highChamberScore(false),
//
//                        // grab completed specimen from human #2
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        doNothing(500),
//                                        //arm.moveTo(35,10,0.5,Arm.PALM_MIDDLE,Arm.CLAW_OPEN),
//                                        //arm.moveTo(0,10,0.5,Arm.PALM_MIDDLE,Arm.CLAW_OPEN)
//                                        new SequentialCommandGroup(
//                                                drive.moveTo(pickup_x, pickup_y, -90).withTimeout(1800)
////                                                drive.turnQuickly(-90).withTimeout(1800)
//                                        )
//                                )
//                        ),
//                        //arm.moveTo(0, 1750, 0.15, 0.5, Arm.CLAW_OPEN),
//                        //arm.moveTo(0, 1750, 0.15, 0.5, Arm.CLAW_CLOSED),
//
//                        // score it
//                        new ParallelCommandGroup(
//                                //arm.highChamber(),
//                                drive.moveTo(-3.1, score_y, -90).withTimeout(2000)
//                        ),
//                        //arm.highChamberScore(true),
//                        //arm.highChamberScore(false),
//
//                        // grab completed specimen from human #3
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        doNothing(500),
//                                        //arm.moveTo(35,10,0.5,0.5,Arm.CLAW_OPEN),
//                                        //arm.moveTo(0,10,0.5,0.5,Arm.CLAW_OPEN)
//                                        new SequentialCommandGroup(
//                                                drive.moveTo(pickup_x, pickup_y, -90).withTimeout(1800)
////                                                drive.turnQuickly(-90).withTimeout(1800)
//                                        )
//                                )
//                        ),
//                        //arm.moveTo(0, 1750, 0.15, 0.5, Arm.CLAW_OPEN),
//                        // arm.moveTo(0, 1750, 0.15, 0.5, Arm.CLAW_CLOSED),
//
//                        // score it
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        drive.moveTo(6.29, score_y, 0).withTimeout(1800)
////                                        drive.turnQuickly(0).withTimeout(1800)
//                                )//,
//                                //arm.highChamber()
//                        ),
//                        //arm.highChamberScore(true),
//                        //arm.highChamberScore(false),
//
//                        // try to park
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        doNothing(500),
//                                        //arm.moveTo(0,1800,0.5,0.5,Arm.CLAW_OPEN)
//                                        new SequentialCommandGroup(
//                                                drive.moveTo(33.4, -64.9, -90).withTimeout(2800)
////                                                drive.turnQuickly(-90).withTimeout(2800)
//                                        )
//                                )
//                        )
//                )
//        );
//    }
//
//    @Override
//    public void loop() {
//        drive.readSensors();
//
//        // Run the CommandScheduler instance
//        CommandScheduler.getInstance().run();
//
//        TelemetryPacket pack = new TelemetryPacket(false);
//        pack.put("Elapsed Time", runtime.toString());
//        pack.put("Time", time);
//        pack.put("Battery", battery.getVoltage());
//        drive.addTelemetry(pack);
//        FtcDashboard.getInstance().sendTelemetryPacket(pack);
//    }
//
//    @Override
//    public void stop() {
//        drive.readSensors();
//        drive.stop();
//

/// /        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
/// /        SharedPreferences.Editor editor = prefs.edit();
/// /        editor.putFloat("heading", (float)drive.getPosition().getHeading(AngleUnit.DEGREES));
/// /        editor.putFloat("x", (float)drive.getPosition().getX(DistanceUnit.METER));
/// /        editor.putFloat("y", (float)drive.getPosition().getY(DistanceUnit.METER));
/// /        editor.putFloat("extension", (float)arm.current_extension_distance);
/// /        editor.putFloat("shoulder_angle",(float) arm.current_shoulder_angle);
/// /        editor.apply();
//
//        CommandScheduler.getInstance().reset();
//    }
//}
