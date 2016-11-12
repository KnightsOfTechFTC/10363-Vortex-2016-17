package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Lego3 on 11/11/2016.
 */
@Autonomous(name = "10363 Competition Autonomous (emergency)")
public class Team10363AutoEmergency extends OpMode {
    Team_10363_Auto_Decs Rob=new Team_10363_Auto_Decs();
    @Override
    public void init() {
        Rob.init(hardwareMap);
    }

    @Override
    public void loop() {
        switch (v_state) {
            case 0:
                Rob.setDrivePower((float) (.5 - Rob.adjspeed(1, Rob.a_gyro_heading())), (float) (.5 + Rob.adjspeed(1, Rob.a_gyro_heading())));
                if (Rob.have_drive_encoders_reached(left_encoder - 5040, right_encoder - 5040, false)) {
                    Rob.setDrivePower(0, 0);
                    left_encoder = Rob.a_left_encoder_pos();
                    right_encoder = Rob.a_right_encoder_pos();
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
