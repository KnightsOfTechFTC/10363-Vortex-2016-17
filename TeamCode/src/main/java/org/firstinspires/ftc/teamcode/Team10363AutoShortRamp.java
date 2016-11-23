package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Lucas on 11/11/2016.
 */

@Autonomous(name = "10363 Competition Autonomous (emergency)")
public class Team10363AutoShortRamp extends OpMode {
    Team_10363_Auto_Decs RobertII=new Team_10363_Auto_Decs();
    @Override
    public void init() {
        RobertII.init(hardwareMap);
    }

    @Override
    public void loop() {
        switch (v_state) {
            case 0: //The first state
                RobertII.setDrivePower((float) (.3), (float) (.3));
                if (RobertII.have_drive_encoders_reached(left_encoder - 4700, right_encoder - 4700, false)) {
                    RobertII.setDrivePower(0, 0);
                    left_encoder = RobertII.a_left_encoder_pos();
                    right_encoder = RobertII.a_right_encoder_pos();
                    v_state++;
                }
                break;
            default:
                break;
        }
    }

    int v_state=0;
    int left_encoder=0;
    int right_encoder=0;

}
