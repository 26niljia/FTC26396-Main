package org.firstinspires.ftc.team26396.opmodes.test;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team26396.constants.ConstantsLoader;

@TeleOp(name = "Test - Constants Load", group = "Test")
@Disabled
public final class ConstantsLoadTest extends CommandOpMode {

    @Override public void initialize() {
        new ConstantsLoader(telemetry).load();
    }
}
