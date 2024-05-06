
// float shootable_angle=10;
float shootable_R_outpost = 0.015f;
float shootable_R_other = 0.02f;
float shootable_R = 0;
float shootable_angle = 5;

float vyaw_threshold = 0.5f;
float delta_R, delta_angle;
enum
{
    gimbal_follow = 0,
    mid_aim = 1,
} vision_mode;
uint8_t last_id_num = 255;
#define outpost_vyaw_level0 1
#define outpost_vyaw_level1 2
float boxxxxxxxxxxx_vision_ctrl[10] = {0};

float lb_1 = 1e-8;
float lb_2 = 1e-8;

uint8_t get_vision_ctrl(float *pitch_ang, float *yaw_ang, float dt)
{
    if (vision_ctrl_data.target_found)
    {
        float yaw = vision_ctrl_data.yaw;
        float r1 = vision_ctrl_data.r1;
        float r2 = vision_ctrl_data.r2;
        float xc = vision_ctrl_data.x;
        float yc = vision_ctrl_data.y;
        float zc = vision_ctrl_data.z;
        float vx = vision_ctrl_data.vx;
        float vy = vision_ctrl_data.vy;
        float vz = vision_ctrl_data.vz;
        float vyaw = vision_ctrl_data.v_yaw;
        float dz = vision_ctrl_data.dz;
        float armor_num = vision_ctrl_data.armor_num;
        float id_num = vision_ctrl_data.id_num;

        if (last_id_num != id_num)
        { // 更新LPF,防止目标切换时不刷新
            r1_filter.fltr_val = r1;
            r2_filter.fltr_val = r2;
            xc_filter.fltr_val = xc;
            yc_filter.fltr_val = yc;
            zc_filter.fltr_val = zc;
            vx_filter.fltr_val = vx;
            vy_filter.fltr_val = vy;
            vz_filter.fltr_val = vz;
            vyaw_filter.fltr_val = vyaw;
            dz_filter.fltr_val = dz;
        }
        last_id_num = id_num;

        if (id_num == 0)
        { // 仅对前哨站进行低通处理,别的简单滤一下就好
            xc_filter.fc = lb_1;
            yc_filter.fc = lb_1;
            zc_filter.fc = lb_1;

            r1_filter.fc = lb_2;
            r2_filter.fc = lb_2;

            // vyaw_filter.fc = 1e-5;
            if (fabs(vyaw - outpost_vyaw_level0 * sign(vyaw)) < 0.5)
                vyaw = outpost_vyaw_level0 * sign(vyaw);
            else if (fabs(vyaw - outpost_vyaw_level1 * sign(vyaw)) < 0.5)
                vyaw = outpost_vyaw_level1 * sign(vyaw);
            shootable_R = shootable_R_outpost;
        }
        else
        { // 击打步兵等目标时shootable_R得单独调
            xc_filter.fc = 1e-3;
            yc_filter.fc = 1e-3;
            zc_filter.fc = 1e-3;

            r1_filter.fc = 1e-7;
            r2_filter.fc = 1e-7;

            vx_filter.fc = 1e-3;
            vy_filter.fc = 1e-3;
            vz_filter.fc = 1e-3;

            vyaw_filter.fc = 1e-4;
            vyaw = LPF_update(&vyaw_filter, vyaw);
            shootable_R = shootable_R_other;
        }

        xc = LPF_update(&xc_filter, xc);
        yc = LPF_update(&yc_filter, yc);
        zc = LPF_update(&zc_filter, zc);
        if (id_num != 0)
        {
            vx = LPF_update(&vx_filter, vx);
            vy = LPF_update(&vy_filter, vy);
            vz = LPF_update(&vz_filter, vz);
        }
        else
            vx = 0, vy = 0, vz = 0;
        dz = sign(dz) * LPF_update(&dz_filter, fabs(dz));

        // r1_fv为高的装甲板半径
        // r2_fv为矮的装甲板半径
        // 不知道为毛if直接判断,r2会寄
        float r1_fv = 0;
        float r2_fv = 0;

        if (dz < 0)
            r1_fv = r1, r2_fv = r2;
        else
            r1_fv = r2, r2_fv = r1;

        r1_fv = LPF_update(&r1_filter, r1_fv);
        r2_fv = LPF_update(&r2_filter, r2_fv);
        float r_short = r1_fv < r2_fv ? r1_fv : r2_fv;

        if (armor_num != 4)
        {
            r2_fv = r1_fv;
            r_short = r1_fv;
            dz = 0;
        }

        // Prediction
        float shoot_delay_offset_t = EXTERNAL_DELAY1 + CONST_SHOOT_DELAY - EXTERNAL_DELAY;
        // param_without_shootdelay=param_within_shootdelay-Vparam*shoot_delay_offset_t;
        // param_without_shootdelay获得的是从弹丸从枪管飞出到正式击打的位置差

        // 以下获得的是从发出开火指令到正式击打的位置差
        xc = xc + predict_time * vx;
        yc = yc + predict_time * vy;
        zc = zc + predict_time * vz;
        yaw = yaw + predict_time * vyaw;

        // 遍历预测后的装甲板,找到哪块装甲板的预测位置最贴近发射方向(需要算上发弹延时)
        float current_yaw = vision_request.yaw;
        float min_yaw_diff = 200000000;
        uint8_t use_1 = 1;
        float diff_angle = 2 * PI / armor_num;
        for (size_t i = 0; i < armor_num; i++)
        {
            float tmp_yaw = yaw + i * diff_angle;
            float yaw_diff = fabs(get_delta_ang(tmp_yaw, current_yaw, 2 * PI));
            if (yaw_diff < min_yaw_diff)
            {
                min_yaw_diff = yaw_diff;
                // float r = use_1 ? r2_fv : r1_fv;
                float r = 0;
                if (dz > 0)
                    r = use_1 ? r2_fv : r1_fv;
                else
                    r = use_1 ? r1_fv : r2_fv;
                best_attack_x = xc - r * cos(tmp_yaw);
                best_attack_y = yc - r * sin(tmp_yaw);
                best_attack_z = use_1 ? zc : zc + dz;
                best_attack_yaw = tmp_yaw;
                best_attack_r = r;
            }
            use_1 = !use_1;
        }

        float theta = atan2(yc, xc);

        if (fabs(vyaw) > vyaw_threshold) //&& vision_ctrl_data.id_num==0  )
        {
            // 前哨站或目标的速度足够,那就瞄准目标中心(不计算发弹延时),等对方来接弹丸
            // 英雄只打半径小的装甲板
            vision_mode = mid_aim;
            est_x = xc - shoot_delay_offset_t * vx - r_short * cos(theta);
            est_y = yc - shoot_delay_offset_t * vy - r_short * sin(theta);

            bool is_best_attack_z_high = ((best_attack_z == zc + dz && dz > 0) || (best_attack_z == zc && dz < 0));

            // 最佳击打的装甲板高度是否较高
            if (best_attack_r == r_short)
            { // 最佳击打装甲板为半径较小的装甲板
                if (is_best_attack_z_high == true)
                { // 半径较小的装甲板较高
                    est_z = best_attack_z - shoot_delay_offset_t * vz;
                }
                else
                { // 半径较小的装甲板较低
                    est_z = best_attack_z - shoot_delay_offset_t * vz;
                }
            }
        }
        else
        { // 目标速度过低,那得自己去追着打
            vision_mode = gimbal_follow;
            est_x = best_attack_x;
            est_y = best_attack_y;
            est_z = best_attack_z;
        }

        // 基于预测装甲板与实际瞄准中心距离差的开火判断代码
        // 理论上开火判断应该在在弹丸出枪管时判断
        // 所以此时就需要将est_x算回发弹延时
        float delta_x = best_attack_x - est_x + shoot_delay_offset_t * vx;
        float delta_y = best_attack_y - est_y + shoot_delay_offset_t * vy;
        float delta_z = best_attack_z - est_z + shoot_delay_offset_t * vz;

        delta_R =
            pow(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z, 0.5f);
        delta_angle =
            get_delta_ang(best_attack_yaw, current_yaw, 2 * PI);
        // 角度控制由于视觉数据可能会不连续,故不推荐加入

        bool is_enter_best_angle = (vision_mode == mid_aim && fabs(delta_angle) < deg2rad(shootable_angle)) || (vision_mode == gimbal_follow);

        if (delta_R < shootable_R && ((vision_mode == mid_aim && best_attack_r == r_short) || vision_mode == gimbal_follow)) //&& is_enter_best_angle
            vision_ctrl_data.suggest_fire = true;
        else
            vision_ctrl_data.suggest_fire = false;

        if (pitch_ang != NULL && yaw_ang != NULL)
        {
            float distance_xy = sqrtf(est_x * est_x + est_y * est_y);

            *yaw_ang = atan2f(est_y, est_x);
            boxxxxxxxxxxx_vision_ctrl[0] = *yaw_ang;
            extern gimbal_state_t vision_ctrl_state, gimbal_real_state;
            boxxxxxxxxxxx_vision_ctrl[1] = vision_ctrl_state.yaw;
            boxxxxxxxxxxx_vision_ctrl[2] = gimbal_real_state.yaw;
            boxxxxxxxxxxx_vision_ctrl[3] = get_delta_ang(vision_ctrl_state.yaw, gimbal_real_state.yaw, 2 * PI);
            target.x0 = distance_xy;
            target.z0 = est_z;
            // target.x0 = 5.6f;
            // target.z0 = 0.54f;
            projectile_solve(cur_v0, &target, &solution);

            if (solution.solution_num > 0)
            {
                if (solution.ang_solution1 < solution.ang_solution2)
                {
                    *pitch_ang = solution.ang_solution1;
                }
                else
                {
                    *pitch_ang = solution.ang_solution2;
                }
            }
            else
            {
                *pitch_ang = atan2f(est_z, distance_xy);
            }
        }
        predict_time += dt;

        return VISION_OK;
    }
    else
    {
        return VISION_NOTARGET;
    }
}