package org.firstinspires.ftc.team26396.opmodes.auto.blue;


// RR-specific imports

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team26396.opmodes.auto.presets.Arm;
import org.firstinspires.ftc.team26396.opmodes.auto.presets.Claw;
import org.firstinspires.ftc.team26396.opmodes.auto.presets.LinearSlide;
import org.firstinspires.ftc.team26396.opmodes.auto.presets.Roll;
import org.firstinspires.ftc.team26396.opmodes.auto.presets.XYaw;
import org.firstinspires.ftc.team26396.opmodes.auto.presets.YPitch;
import org.firstinspires.ftc.team26396.roadrunner.teamcode.MecanumDrive;

@Autonomous(name="Blue High Basket Drop Auto Path")
public class BlueHighBasketDropAutoPath extends LinearOpMode {
    public void runOpMode() {
        // I'm assuming you're at 0, 60, facing the basket
        Pose2d initialPose = new Pose2d(0, 60, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // Define arm positions using the constants from the Arm class

        Arm arm = new Arm(hardwareMap);
        LinearSlide linearSlide = new LinearSlide(hardwareMap);

        XYaw yaw = new XYaw(hardwareMap);
        YPitch pitch = new YPitch(hardwareMap);
        Roll roll = new Roll(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        waitForStart();

        double xDestPositionDropSampleInHand = 52;
        double yDestPositionDropSampleInHand = 52;
        double headingDestPositionDropSampleInHand = Math.toRadians(0.0);

        TrajectoryActionBuilder goToBasketFromInitPosition = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(xDestPositionDropSampleInHand, yDestPositionDropSampleInHand), Math.toRadians(45));


        Action sleepAction1Second = new SleepAction(1);
        Action sleepAction2Seconds = new SleepAction(2);
        Action sleepAction3Seconds = new SleepAction(3);

        TrajectoryActionBuilder basketToFirstSample =
                goToBasketFromInitPosition.endTrajectory().fresh()
                        .turnTo(Math.toRadians(-90))
                        .lineToY(38);

        TrajectoryActionBuilder firstSampleToBasket =
                basketToFirstSample.endTrajectory().fresh()
                        .strafeToLinearHeading(new Vector2d(xDestPositionDropSampleInHand, yDestPositionDropSampleInHand), Math.toRadians(45));

        Action goToBasketFromInitPositionAction = goToBasketFromInitPosition.build();
        Action basketToFirstSampleAction = basketToFirstSample.build();
        Action firstSampleToBasketAction = firstSampleToBasket.build();

        Action fullAction = new SequentialAction(
                // Init position to basket with pre-loaded sample
                goToBasketFromInitPositionAction,
                // Drop the sample into the basket
                buildCommonActionForDroppingToBasket(goToBasketFromInitPosition, arm, linearSlide, claw, pitch),
                // Go from basket to the first sample
                firstSampleToBasketAction,
                // Pick up first sample
                buildCommonActionToPickSample(basketToFirstSample, arm, linearSlide, claw, pitch),
                // Drop the sample into the basket
                buildCommonActionForDroppingToBasket(goToBasketFromInitPosition, arm, linearSlide, claw, pitch));
                // Go from basket to the second sample

        Actions.runBlocking(fullAction);

    }

    public Action buildCommonActionForDroppingToBasket(TrajectoryActionBuilder inputTrajectory, Arm arm, LinearSlide linearSlide, Claw claw, YPitch pitch) {

        Action dropSampleIntoHighBasketAction = inputTrajectory.endTrajectory().fresh()
                .stopAndAdd(new SleepAction(1))
                .stopAndAdd(arm.raiseArmForUpperBasket())
                .stopAndAdd(new SleepAction(1))
                .stopAndAdd(linearSlide.extendArmForward())
                .stopAndAdd(new SleepAction(1))
                .stopAndAdd(pitch.moveWristUp())
                .stopAndAdd(new SleepAction(1))
                .stopAndAdd(claw.openClaw())
                .stopAndAdd(new SleepAction(1))
                .stopAndAdd(pitch.moveWristDown())
                .stopAndAdd(new SleepAction(1))
                .stopAndAdd(linearSlide.retractSlideBackward())
                .stopAndAdd(new SleepAction(3))
                .stopAndAdd(arm.deactivateArm())
                .stopAndAdd(new SleepAction(2))
                .build();

        return dropSampleIntoHighBasketAction;
    }

    public Action buildCommonActionToPickSample(TrajectoryActionBuilder inputTrajectory, Arm arm, LinearSlide linearSlide, Claw claw, YPitch pitch) {

        Action dropSampleIntoHighBasketAction = inputTrajectory.endTrajectory().fresh()
                .stopAndAdd(linearSlide.moveSlideRelatively(50))
                .stopAndAdd(claw.openClaw())
                .stopAndAdd(new SleepAction(1))
                .stopAndAdd(claw.closeClaw())
                .stopAndAdd(new SleepAction(1))
                .stopAndAdd(linearSlide.moveSlideRelatively(-50))
                .build();

        return dropSampleIntoHighBasketAction;
    }
}