package org.firstinspires.ftc.teamcode.opModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "LeftBluePushAuto", group = "Real")
public final class LeftBluePushAuto extends LinearOpMode {
    private  MecanumDrive drive;

    @Override
    public void runOpMode()  {
        Pose2d beginPose = new Pose2d(41,65 , Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)

                        .strafeToConstantHeading( new Vector2d(62, 55))

                        .strafeToConstantHeading(new Vector2d(35,35))
                        .splineToConstantHeading(new Vector2d(45, 20),Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(58, 64) , Math.toRadians(90))
                        .build());
    }

    public class updatePose implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            drive.updatePoseEstimate();
            return false;
        }
    }
    public Action updatePose(MecanumDrive drive) {
        return new updatePose();
    }
}
