package org.firstinspires.ftc.teamcode.util.nulldevices;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import androidx.annotation.NonNull;

public class NullDrive extends SampleMecanumDrive {
    public NullDrive() {
        super(null);
    }
}
