package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "RightBluePushAuto", group = "Real")
public final class RightBluePushAuto extends LinearOpMode {
    @Override
    public void runOpMode()  {
        //Pose2d beginPose = new Pose2d(-12,65 , 3*Math.PI/2);
        Pose2d beginPose = new Pose2d(-12,65 , Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
            drive.actionBuilder(beginPose)

                    .strafeToConstantHeading( new Vector2d( -12, 60) )
                .strafeToConstantHeading( new Vector2d( -48, 60) )
                    .strafeToConstantHeading(new Vector2d(-36,30))
                    .splineToConstantHeading(new Vector2d(-42,15),Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(-48,24),Math.toRadians(90))

                .build());
    }
}
