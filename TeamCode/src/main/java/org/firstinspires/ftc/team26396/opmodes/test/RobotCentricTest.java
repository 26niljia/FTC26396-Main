package org.firstinspires.ftc.team26396.opmodes.test;

import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team26396.subsystems.drive.MecanumDriveSubsystem;
import org.firstinspires.ftc.team26396.subsystems.drive.commands.RobotCentricCommand;

@TeleOp(name = "Test - Robot Centric", group = "Test")
@Disabled
public final class RobotCentricTest extends CommandOpMode {
    GamepadEx driverGamepad;

    @Override public void initialize() {
        MecanumDriveSubsystem driveSubsystem = new MecanumDriveSubsystem(this);
        driverGamepad       = new GamepadEx(gamepad1);

        register(driveSubsystem);

        schedule(
                new RobotCentricCommand(
                        driveSubsystem,
                        driverGamepad::getLeftY,
                        driverGamepad::getLeftX,
                        driverGamepad::getRightX
                )
        );
    }
}