package org.firstinspires.ftc.teamcode;

/**
 * Created by Lego5 on 11/11/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Lego5 on 11/11/2016.
 */
@Autonomous(name="10363 Competition Autonomous on Red Alliance (no beacon)")
public class Team10363AutoRedNeither extends OpMode {
    Team_10363_Auto_Decs RobII = new Team_10363_Auto_Decs();

    @Override
    public void init() {
        RobII.init(hardwareMap);
        left_encoder = RobII.a_left_encoder_pos();
        right_encoder = RobII.a_right_encoder_pos();
        telemetry.addData("4: Heading", RobII.a_gyro_heading());
        telemetry.addData("6: Ground Color (Alpha)", RobII.a_ground_alpha());
        telemetry.update();
    }

    @Override
    public void loop() {
        switch (v_state) {
            case 0:
                RobII.setDrivePower((float) (.5 - RobII.adjspeed(1, RobII.a_gyro_heading())), (float) (.5 + RobII.adjspeed(1, RobII.a_gyro_heading())));
                if (RobII.have_drive_encoders_reached(left_encoder - 4220, right_encoder - 4220, false)) {
                    RobII.setDrivePower(0, 0);
                    left_encoder = RobII.a_left_encoder_pos();
                    right_encoder = RobII.a_right_encoder_pos();
                    v_state++;
                }
                break;
            case 1:
                RobII.setDrivePower((float) -.4, 0);
                if (RobII.a_gyro_heading() <= 330 && RobII.a_gyro_heading() > 270) {
                    RobII.setDrivePower(0, 0);
                    left_encoder = RobII.a_left_encoder_pos();
                    right_encoder = RobII.a_right_encoder_pos();
                    v_state++;
                }
                break;
            case 2:
                RobII.setDrivePower((float) (.3 - RobII.adjspeed(1, RobII.a_gyro_heading() - 315)), (float) (.3 + RobII.adjspeed(1, RobII.a_gyro_heading() - 315)));
                if (RobII.have_drive_encoders_reached(left_encoder - 6000, right_encoder - 6000, false)) {
                    RobII.setDrivePower(0, 0);
                    left_encoder = RobII.a_left_encoder_pos();
                    right_encoder = RobII.a_right_encoder_pos();
                    v_state++;
                }
                break;

        }

    }
    int v_state = 0;
    int left_encoder = 0;
    int right_encoder = 0;
    int encoderLeft;
    int encoderRight;
    double bblue = 0;
    double bred = 0;
    int count = 0;
    int leftEncoderProblems = 0;
    int rightEncoderProblems = 0;
}
