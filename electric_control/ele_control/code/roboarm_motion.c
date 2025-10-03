#include "roboarm_motion.h"
#include <stdio.h>
#include "zf_common_headfile.h"

// 褰撳墠浣嶇疆鍏ㄥ眬鍙橀噺
static ServoPositions current_pos = {
    .di = 0,
    .dabi = 0,
    .zhongbi = 0,
    .xiaobi = 0,
    .shouwan = 0,
    .gripper = 0};

static const ServoPositions RESET_POS = {
    .di = 510,
    .dabi = 540,
    .zhongbi = 1240,
    .xiaobi = 370,
    .shouwan = 680,
    .gripper = GRIPPER_OPEN};

static const ServoPositions RESET_POS_2 = {
    .di = 970,
    .dabi = 320,
    .zhongbi = 740,
    .xiaobi = 370,
    .shouwan = 680,
    .gripper = GRIPPER_OPEN};

static const ServoPositions PREPARE_POS_0 = {
    .di = 970,
    .dabi = 530,
    .zhongbi = 540,
    .xiaobi = 1060,
    .shouwan = 680,
    .gripper = GRIPPER_OPEN};

static const ServoPositions PREPARE_POS = {
    .di = 970,
    .dabi = 560,
    .zhongbi = 580,
    .xiaobi = 1060,
    .shouwan = 680,
    .gripper = GRIPPER_OPEN};

static const ServoPositions PREPARE_POS_1 = {
    .di = 970,
    .dabi = 630,
    .zhongbi = 670,
    .xiaobi = 1080,
    .shouwan = 680,
    .gripper = GRIPPER_OPEN};

static const ServoPositions PREPARE_POS_2 = {
    .di = 970,
    .dabi = 665,
    .zhongbi = 750,
    .xiaobi = 1060,
    .shouwan = 680,
    .gripper = GRIPPER_OPEN};

static const ServoPositions PREPARE_POS_3 = {
    .di = 970,
    .dabi = 710,
    .zhongbi = 830,
    .xiaobi = 1060,
    .shouwan = 680,
    .gripper = GRIPPER_OPEN};

static const ServoPositions PREPARE_POS_4 = {
    .di = 970,
    .dabi = 720,
    .zhongbi = 940,
    .xiaobi = 1040,
    .shouwan = 680,
    .gripper = GRIPPER_OPEN}; // 澶ц噦涓噦鍙傛暟杈冧箣鍓�-30锛屽簲瀵规姄鐖埖鏈烘敼鏈濅笂鏀剧疆

static const ServoPositions GRASP_POS = {
    .di = 970,
    .dabi = 710,
    .zhongbi = 800,
    .xiaobi = 1060,
    .shouwan = 680,
    .gripper = GRIPPER_CLOSE};

static const ServoPositions HIGH_PREPARE_POS = {
    .di = 970,
    .dabi = 550,
    .zhongbi = 790,
    .xiaobi = 850,
    .shouwan = 680,
    .gripper = GRIPPER_OPEN};

static const ServoPositions HIGH_GRASP_POS = {
    .di = 970,
    .dabi = 700,
    .zhongbi = 1080,
    .xiaobi = 780,
    .shouwan = 680,
    .gripper = GRIPPER_CLOSE};

static const ServoPositions HIGH_GRASP_POS_2 = {
    .di = 970,
    .dabi = 770,
    .zhongbi = 1220,
    .xiaobi = 700,
    .shouwan = 680,
    .gripper = GRIPPER_CLOSE};

static const ServoPositions STORE_POS = {
    .di = 620,
    .dabi = 580,
    .zhongbi = 1030,
    .xiaobi = 390,
    .shouwan = 680,
    .gripper = GRIPPER_CLOSE};

static const ServoPositions SHOT_POS = {
    .di = 510,
    .dabi = 570,
    .zhongbi = 1040,
    .xiaobi = 370,
    .shouwan = 680,
    .gripper = GRIPPER_CLOSE};

static void arm_set_pwm(ServoPositions pos)
{
    current_pos = pos;
    pwm_set_duty(DI_PWM, pos.di);
    pwm_set_duty(DABI_PWM, pos.dabi);
    pwm_set_duty(ZHONGBI_PWM, pos.zhongbi);
    pwm_set_duty(XIAOBI_PWM, pos.xiaobi);
    pwm_set_duty(SHOUWAN_PWM, pos.shouwan);
    pwm_set_duty(GRIPPER_PWM, pos.gripper);
}

static void move_pose_smoothly(ServoPositions target_pos, int16_t time_ms)
{
    // 璁＄畻姣忎釜鑸垫満鐨勭Щ鍔ㄨ窛绂�
    int diff_di = target_pos.di - current_pos.di;
    int diff_dabi = target_pos.dabi - current_pos.dabi;
    int diff_zhongbi = target_pos.zhongbi - current_pos.zhongbi;
    int diff_xiaobi = target_pos.xiaobi - current_pos.xiaobi;
    int diff_shouwan = target_pos.shouwan - current_pos.shouwan;
    int diff_gripper = target_pos.gripper - current_pos.gripper;

    // 璁＄畻姝ユ暟锛堝熀浜庢椂闂村拰寤惰繜锛�
    int total_steps = time_ms / SMOOTH_DELAY_MS;
    if (total_steps <= 0)
        total_steps = 1;

    // 鐖瓙鐙珛姝ユ暟璁＄畻锛�300ms鍐呭畬鎴愶級
    int gripper_steps = GRIPPER_MOVE_TIME_MS / SMOOTH_DELAY_MS;
    if (gripper_steps <= 0)
        gripper_steps = 1;
    if (gripper_steps > total_steps)
        gripper_steps = total_steps;

    // 璁＄畻姣忔鐨勭Щ鍔ㄩ噺锛堝叾浠栧叧鑺傦級
    float step_di = (float)diff_di / total_steps;
    float step_dabi = (float)diff_dabi / total_steps;
    float step_zhongbi = (float)diff_zhongbi / total_steps;
    float step_xiaobi = (float)diff_xiaobi / total_steps;
    float step_shouwan = (float)diff_shouwan / total_steps;

    // 璁＄畻鐖瓙姣忔绉诲姩閲忥紙鐙珛閫熷害锛�
    float step_gripper = (float)diff_gripper / gripper_steps;

    // 璁板綍璧峰浣嶇疆
    ServoPositions start_pos = current_pos;

    // 閫愭绉诲姩鍒扮洰鏍囦綅缃�
    for (int step = 1; step <= total_steps; step++)
    {
        // 璁＄畻褰撳墠姝ョ殑鐩爣浣嶇疆锛堝叾浠栧叧鑺傦級
        int target_di = start_pos.di + (int)(step_di * step);
        int target_dabi = start_pos.dabi + (int)(step_dabi * step);
        int target_zhongbi = start_pos.zhongbi + (int)(step_zhongbi * step);
        int target_xiaobi = start_pos.xiaobi + (int)(step_xiaobi * step);
        int target_shouwan = start_pos.shouwan + (int)(step_shouwan * step);

        // 璁＄畻鐖瓙浣嶇疆锛堢嫭绔嬫椂闂存帶鍒讹級
        int target_gripper;
        if (step <= gripper_steps)
        {
            // 鐖瓙鍦ㄥ墠gripper_steps姝ュ唴瀹屾垚绉诲姩
            target_gripper = start_pos.gripper + (int)(step_gripper * step);
        }
        else
        {
            // 鐖瓙宸茬粡鍒拌揪鐩爣锛屼繚鎸佷綅缃�
            target_gripper = target_pos.gripper;
        }

        // 鍦ㄦ渶鍚庝竴姝ワ紝纭繚鎵�鏈夊叧鑺傚埌杈剧簿纭殑鐩爣浣嶇疆
        if (step == total_steps)
        {
            target_di = target_pos.di;
            target_dabi = target_pos.dabi;
            target_zhongbi = target_pos.zhongbi;
            target_xiaobi = target_pos.xiaobi;
            target_shouwan = target_pos.shouwan;
            target_gripper = target_pos.gripper;
        }

        // 鏇存柊褰撳墠浣嶇疆骞惰缃甈WM
        current_pos.di = target_di;
        current_pos.dabi = target_dabi;
        current_pos.zhongbi = target_zhongbi;
        current_pos.xiaobi = target_xiaobi;
        current_pos.shouwan = target_shouwan;
        current_pos.gripper = target_gripper;

        arm_set_pwm(current_pos);

        // 寤惰繜
        system_delay_ms(SMOOTH_DELAY_MS);
    }
}

static void delay_step(void)
{
    system_delay_ms(STEP_DELAY_MS);
}

void roboarm_init(void)
{
    pwm_init(DI_PWM, 50, 0);
    pwm_init(DABI_PWM, 50, 0);
    pwm_init(ZHONGBI_PWM, 50, 0);
    pwm_init(XIAOBI_PWM, 50, 0);
    pwm_init(SHOUWAN_PWM, 50, 0);
    pwm_init(GRIPPER_PWM, 50, 0);
}

void arm_reset(void)
{
    if (current_pos.di == 0 ||
        current_pos.dabi == 0 ||
        current_pos.zhongbi == 0 ||
        current_pos.xiaobi == 0 ||
        current_pos.shouwan == 0 ||
        current_pos.gripper == 0)
    {
        current_pos = RESET_POS_2;
        arm_set_pwm(RESET_POS_2);
        move_pose_smoothly(RESET_POS_2, 500);
    }
    else
    {
        // current_pos = RESET_POS;
        move_pose_smoothly(RESET_POS_2, 500);
    }
}

void arm_relax(void)
{
    current_pos.di = 0;
    current_pos.dabi = 0;
    current_pos.zhongbi = 0;
    current_pos.xiaobi = 0;
    current_pos.shouwan = 0;
    current_pos.gripper = 0;

    arm_set_pwm(current_pos);
}

void arm_reset_to_prepare(void)
{
    printf("=== Starting reset to prepare ===\r\n");

    // 1) 搴曞骇鏃嬭浆鍒板噯澶囦綅缃�
    ServoPositions p = current_pos;
    p.xiaobi = PREPARE_POS.xiaobi;
    printf("Step 1: Moving base to prepare position\r\n");
    move_pose_smoothly(p, 500);
    delay_step();

    // 2) 鍏朵粬鑷傚悓姝ュ钩婊戞棆杞埌鍑嗗浣嶇疆
    p = PREPARE_POS;
    printf("Step 2: Moving all arms to prepare position\r\n");
    move_pose_smoothly(p, 1000);
    delay_step();

    printf("=== Reset to prepare complete ===\r\n");
}

void arm_reset_to_high_prepare(void)
{
    printf("=== Starting reset to prepare ===\r\n");

    // 1) 搴曞骇鏃嬭浆鍒板噯澶囦綅缃�
    ServoPositions p = current_pos;
    p.di = HIGH_PREPARE_POS.di;
    printf("Step 1: Moving base to prepare position\r\n");
    move_pose_smoothly(p, 500);
    delay_step();

    // 2) 鍏朵粬鑷傚悓姝ュ钩婊戞棆杞埌鍑嗗浣嶇疆
    p = HIGH_PREPARE_POS;
    printf("Step 2: Moving all arms to prepare position\r\n");
    move_pose_smoothly(p, 1000);
    delay_step();

    printf("=== Reset to prepare complete ===\r\n");
}

void arm_prepare_to_grip(void)
{
    printf("=== Starting prepare to grip ===\r\n");

    // 1) 鍚屾灏嗗ぇ鑷�/涓噦绉诲姩鍒版姄鍙栦綅锛堝叾浠栧叧鑺備繚鎸佷笉鍙橈級
    ServoPositions p = current_pos;
    p = PREPARE_POS_2;
    printf("Step 1: Moving arm to grip position\r\n");
    move_pose_smoothly(p, 800);
    delay_step();

    // 2) 澶圭埅闂悎褰㈡垚鎶撳彇
    printf("Step 2: Closing gripper\r\n");
    p = current_pos;
    p.gripper = GRIPPER_CLOSE;
    move_pose_smoothly(p, 300);
    delay_step();

    printf("=== Prepare to grip complete ===\r\n");
}

void arm_high_prepare_to_grip(void)
{
    printf("=== Starting prepare to grip ===\r\n");

    // 1) 鍚屾灏嗗ぇ鑷�/涓噦绉诲姩鍒版姄鍙栦綅锛堝叾浠栧叧鑺備繚鎸佷笉鍙橈級
    ServoPositions p = current_pos;
    p.dabi = HIGH_GRASP_POS.dabi;
    p.zhongbi = HIGH_GRASP_POS.zhongbi;
    p.xiaobi = HIGH_GRASP_POS.xiaobi;
    printf("Step 1: Moving arm to grip position\r\n");
    move_pose_smoothly(p, 800);
    delay_step();

    // 2) 澶圭埅闂悎褰㈡垚鎶撳彇
    printf("Step 2: Closing gripper\r\n");
    p = current_pos;
    p.gripper = GRIPPER_CLOSE;
    move_pose_smoothly(p, 300);
    delay_step();

    printf("=== Prepare to grip complete ===\r\n");
}

void arm_grip_to_shot(void)
{
    printf("=== Starting grip to shot ===\r\n");

    ServoPositions p;

    // 1) 澶ц噦鍒版姇鎺蜂綅锛屽噯澶囨姇鎺�
    p = current_pos;
    p.dabi = SHOT_POS.dabi;
    p.zhongbi = PROCESS2_ZHONGBI;
    printf("Step 1: Moving to shot dabi position\r\n");
    move_pose_smoothly(p, 600);
    delay_step();

    // 3) 涓噦鍒拌繃绋嬩綅锛屽簳搴у埌鎶曟幏浣�
    p = current_pos;
    p.zhongbi = PROCESS_ZHONGBI;
    p.di = SHOT_POS.di;
    p.xiaobi = SHOT_POS.xiaobi;
    printf("Step 4: Moving to process zhongbi position\r\n");
    move_pose_smoothly(p, 700);
    delay_step();

    // 4) 涓噦鍒版姇鎺蜂綅锛屽噯澶囨斁椋為晼
    p = current_pos;
    p.zhongbi = SHOT_POS.zhongbi;
    printf("Step 6: Moving to shot zhongbi position for dart placement\r\n");
    move_pose_smoothly(p, 1000);
    delay_step();

    // 5) 鎵撳紑澶圭埅锛屾斁椋為晼
    p = current_pos;
    p.gripper = GRIPPER_OPEN;
    p.zhongbi = PROCESS_ZHONGBI;
    printf("Step 7: Opening gripper\r\n");
    move_pose_smoothly(p, 500);
    delay_step();

    // // 6) 鍥炲埌绛夊緟浣嶇疆锛堝彲閫夛級
    // p = current_pos;
    // p.zhongbi = PROCESS_ZHONGBI;
    // printf("Step 8: Moving to wait position\r\n");
    // move_pose_smoothly(p, 400);
    // delay_step();

    printf("=== Grip to shot complete ===\r\n");
}

void arm_grip_to_store(void)
{
    printf("=== Starting grip_to_store ===\r\n");

    ServoPositions p;

    // 1) 澶ц噦鎶曟幏浣嶏紝鍑嗗鎶曟幏
    p = current_pos;
    p.dabi = STORE_POS.dabi;
    p.zhongbi = PROCESS2_ZHONGBI;
    printf("Step 1: Moving to shot dabi position\r\n");
    move_pose_smoothly(p, 700);
    delay_step();
    //
    //    // 2) 灏忚噦鍒版姇鎺蜂綅
    //    p = current_pos;
    //    p.xiaobi = STORE_POS.xiaobi;
    //    printf("Step 3: Moving to shot xiaobi position\r\n");
    //    move_pose_smoothly(p, 500);
    //    delay_step();

    // 3) 涓噦鍒拌繃绋嬩綅锛屽簳搴у埌鎶曟幏浣�
    p = current_pos;
    p.zhongbi = PROCESS_ZHONGBI;
    p.di = STORE_POS.di;
    p.xiaobi = STORE_POS.xiaobi;
    printf("Step 4: Moving to process zhongbi position\r\n");
    move_pose_smoothly(p, 800);
    delay_step();

    // 4) 涓噦鍒版姇鎺蜂綅锛屽噯澶囨斁椋為晼
    p = current_pos;
    p.zhongbi = STORE_POS.zhongbi;
    printf("Step 6: Moving to shot zhongbi position for dart placement\r\n");
    move_pose_smoothly(p, 1000);
    delay_step();

    // 5) 鎵撳紑澶圭埅锛屾斁椋為晼
    p = current_pos;
    p.gripper = GRIPPER_OPEN;
    p.zhongbi = PROCESS_ZHONGBI;
    printf("Step 7: Opening gripper\r\n");
    move_pose_smoothly(p, 500);
    delay_step();

    printf("=== Grip to shot complete ===\r\n");
}

void arm_grip_to_wait_shot(void)
{
    printf("=== Starting grip to shot ===\r\n");

    ServoPositions p;

    // 1) 澶ц噦鎶曟幏浣嶏紝鍑嗗鎶曟幏
    p = current_pos;
    p.xiaobi = PROCESS_XIAOBI;
    printf("Step 1: Moving to shot dabi position\r\n");
    move_pose_smoothly(p, 600);
    delay_step();

    // 3) 涓噦鍒拌繃绋嬩綅锛屽簳搴у埌鎶曟幏浣�
    p = current_pos;
    p.dabi = SHOT_POS.dabi;
    p.zhongbi = PROCESS3_ZHONGBI;
    printf("Step 4: Moving to process zhongbi position\r\n");
    move_pose_smoothly(p, 600);
    delay_step();

    p = current_pos;
    p.xiaobi = SHOT_POS.xiaobi;
    p.zhongbi = PROCESS_ZHONGBI;
    p.di = SHOT_POS.di;
    printf("Step 4: Moving to process zhongbi position\r\n");
    move_pose_smoothly(p, 800);
    delay_step();

    printf("=== Grip to shot complete ===\r\n");
}

void arm_wait_shot_to_shot(void)
{
    printf("=== Starting grip to shot ===\r\n");

    ServoPositions p;
    // 5) 鎵撳紑澶圭埅锛屾斁椋為晼

    // 4) 涓噦鍒版姇鎺蜂綅锛屽噯澶囨斁椋為晼
    p = current_pos;
    p.zhongbi = SHOT_POS.zhongbi;
    printf("Step 6: Moving to shot zhongbi position for dart placement\r\n");
    move_pose_smoothly(p, 800);
    delay_step();

    p = current_pos;
    p.gripper = GRIPPER_OPEN;
    p.zhongbi = PROCESS_ZHONGBI;
    printf("Step 7: Opening gripper\r\n");
    move_pose_smoothly(p, 500);
    delay_step();
}

void arm_high_grip_to_shot(void)
{
    printf("=== Starting grip to shot ===\r\n");

    ServoPositions p;

    // 1) 澶ц噦鎶曟幏浣嶏紝鍑嗗鎶曟幏
    p = current_pos;
    p.xiaobi = PROCESS_XIAOBI;
    printf("Step 1: Moving to shot dabi position\r\n");
    move_pose_smoothly(p, 600);
    delay_step();

    // 3) 涓噦鍒拌繃绋嬩綅锛屽簳搴у埌鎶曟幏浣�
    p = current_pos;
    p.dabi = SHOT_POS.dabi;
    p.zhongbi = PROCESS3_ZHONGBI;
    printf("Step 4: Moving to process zhongbi position\r\n");
    move_pose_smoothly(p, 600);
    delay_step();

    p = current_pos;
    p.xiaobi = SHOT_POS.xiaobi;
    p.zhongbi = PROCESS_ZHONGBI;
    p.di = SHOT_POS.di;
    printf("Step 4: Moving to process zhongbi position\r\n");
    move_pose_smoothly(p, 800);
    delay_step();

    // 4) 涓噦鍒版姇鎺蜂綅锛屽噯澶囨斁椋為晼
    p = current_pos;
    p.zhongbi = SHOT_POS.zhongbi;
    printf("Step 6: Moving to shot zhongbi position for dart placement\r\n");
    move_pose_smoothly(p, 1000);
    delay_step();

    // 5) 鎵撳紑澶圭埅锛屾斁椋為晼
    p = current_pos;
    p.gripper = GRIPPER_OPEN;
    p.zhongbi = PROCESS_ZHONGBI;
    printf("Step 7: Opening gripper\r\n");
    move_pose_smoothly(p, 500);
    delay_step();

    printf("=== Grip to shot complete ===\r\n");
}

void arm_shot_to_reset(void)
{
    printf("=== Starting shot to reset ===\r\n");

    ServoPositions p;

    // 1) 鍥炲埌澶嶄綅浣嶇疆锛屽噯澶囦笅涓�娆℃姄鍙�
    //    p = RESET_POS;
    //    printf("Step 1: Moving to grip di position\r\n");
    //    move_pose_smoothly(p, 600);
    //    delay_step();

    p = RESET_POS_2;
    printf("Step 1: Moving to grip di position\r\n");
    move_pose_smoothly(p, 600);
    delay_step();

    printf("=== Shot to grip prepare complete ===\r\n");
}

void arm_store_to_reset(void)
{
    printf("=== Starting shot to reset ===\r\n");

    ServoPositions p;

    p = RESET_POS_2;
    printf("Step 1: Moving to grip di position\r\n");
    move_pose_smoothly(p, 600);
    delay_step();

    printf("=== Shot to grip prepare complete ===\r\n");
}

void arm_reset_to_store(void)
{
    printf("=== Starting shot to reset ===\r\n");

    ServoPositions p;

    p = STORE_POS;
    p.zhongbi = PROCESS_ZHONGBI;
    p.gripper = GRIPPER_OPEN;
    printf("Step 1: \r\n");
    move_pose_smoothly(p, 600);
    delay_step();

    p = current_pos;
    p.zhongbi = STORE_POS.zhongbi;
    printf("Step 2: \r\n");
    move_pose_smoothly(p, 600);
    delay_step();

    p = current_pos;
    p.gripper = GRIPPER_CLOSE;
    printf("Step 3: \r\n");
    move_pose_smoothly(p, 600);
    delay_step();
    printf("=== Shot to grip prepare complete ===\r\n");
}

void arm_store_to_shot(void)
{
    printf("=== Starting grip_to_store ===\r\n");

    ServoPositions p;

    p = current_pos;
    p.dabi = SHOT_POS.dabi;
    p.zhongbi = PROCESS_ZHONGBI;
    printf("Step 1: Moving to shot dabi position\r\n");
    move_pose_smoothly(p, 600);
    delay_step();

    p = current_pos;
    p.di = SHOT_POS.di;
    printf("Step 4: Moving to process zhongbi position\r\n");
    move_pose_smoothly(p, 400);
    delay_step();

    p = current_pos;
    p.zhongbi = SHOT_POS.zhongbi;
    printf("Step 6: Moving to shot zhongbi position for dart placement\r\n");
    move_pose_smoothly(p, 800);
    delay_step();

    p = current_pos;
    p.gripper = GRIPPER_OPEN;
    p.zhongbi = PROCESS_ZHONGBI;
    printf("Step 7: Opening gripper\r\n");
    move_pose_smoothly(p, 500);
    delay_step();

    printf("=== Grip to shot complete ===\r\n");
}

void arm_high_grip_to_store(void)
{
    printf("=== Starting grip to shot ===\r\n");

    ServoPositions p;

    // 1) 澶ц噦鎶曟幏浣嶏紝鍑嗗鎶曟幏
    p = current_pos;
    p.xiaobi = PROCESS_XIAOBI;
    printf("Step 1: Moving to shot dabi position\r\n");
    move_pose_smoothly(p, 600);
    delay_step();

    // 3) 涓噦鍒拌繃绋嬩綅锛屽簳搴у埌鎶曟幏浣�
    p = current_pos;
    p.dabi = STORE_POS.dabi;
    p.zhongbi = PROCESS3_ZHONGBI;
    printf("Step 4: Moving to process zhongbi position\r\n");
    move_pose_smoothly(p, 600);
    delay_step();

    p = current_pos;
    p.xiaobi = STORE_POS.xiaobi;
    p.zhongbi = PROCESS_ZHONGBI;
    p.di = STORE_POS.di;
    printf("Step 4: Moving to process zhongbi position\r\n");
    move_pose_smoothly(p, 600);
    delay_step();

    // 4) 涓噦鍒版姇鎺蜂綅锛屽噯澶囨斁椋為晼
    p = current_pos;
    p.zhongbi = STORE_POS.zhongbi;
    printf("Step 6: Moving to shot zhongbi position for dart placement\r\n");
    move_pose_smoothly(p, 900);
    delay_step();

    // 5) 鎵撳紑澶圭埅锛屾斁椋為晼
    p = current_pos;
    p.gripper = GRIPPER_OPEN;
    p.zhongbi = PROCESS_ZHONGBI;
    printf("Step 7: Opening gripper\r\n");
    move_pose_smoothly(p, 500);
    delay_step();

    printf("=== Grip to shot complete ===\r\n");
}
