package org.firstinspires.ftc.teamcode.OpModes.TELEOP;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Subsystems.LiftArm;

@Config
@TeleOp(name = "Mario Party 8")
public class MarioParty extends OpMode {

    public static double LIFT_EXTENSION = 0;
    public static double SHOULDER_ANGLE = 0;
    public static double WRIST_ANGLE = 0;
    public static double CLAW_STATE = 0;
    GamepadEx controller;
    LiftArm liftArm;
    VoltageSensor battery;

    public void init() {
        CommandScheduler.getInstance().reset();

        liftArm = new LiftArm(hardwareMap);
        battery = hardwareMap.voltageSensor.get("Control Hub");
        controller = new GamepadEx(gamepad2);

        // Register Subsystem objects to the scheduler
        CommandScheduler.getInstance().registerSubsystem(liftArm);
        liftArm.reset();
    }

    public void init_loop() {
        liftArm.read_sensors();
        liftArm.periodic();
    }

    @Override
    public void loop() {
        controller.readButtons();
        liftArm.read_sensors();

        if (controller.wasJustPressed(GamepadKeys.Button.Y)) {
            liftArm.moveTo(SHOULDER_ANGLE,LIFT_EXTENSION,WRIST_ANGLE,CLAW_STATE);
        }
    }
}
