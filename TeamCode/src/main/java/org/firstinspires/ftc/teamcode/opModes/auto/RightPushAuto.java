package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.roadrunner.MecanumDrive;

@Autonomous(name = "RightPushAuto", group = "Real")
public final class RightPushAuto extends LinearOpMode {
    private  MecanumDrive drive;

    @Override
    public void runOpMode()  {
        Pose2d beginPose = new Pose2d(-16.5,65 , Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)

                        .strafeToConstantHeading( new Vector2d(-35.75, 60) )
                        .strafeToConstantHeading(new Vector2d(-35.75,30))

                        .splineToConstantHeading(new Vector2d(-47.6,16),Math.toRadians(180))
                        .strafeToConstantHeading(new Vector2d( -47.6, 61))
                        .setTangent(Math.toRadians(270))

                        .splineToConstantHeading(new Vector2d(-56.75,16),Math.toRadians(180))
                        .strafeToConstantHeading(new Vector2d(-56.75,61))

                        .setTangent(Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(-62, 20), Math.toRadians(180))
                        .strafeToConstantHeading(new Vector2d(-62, 61))

                        .splineToLinearHeading(new Pose2d(-48, 61 ,Math.toRadians(180)), Math.toRadians(180))
                        .build());
    }
}