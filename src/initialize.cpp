#include "main.h"
#include "Robot.h"
using namespace pros;

LV_IMG_DECLARE(logo1);
lv_obj_t * myButton;
lv_obj_t * myButtonLabel;
lv_obj_t * myLabel;

lv_style_t myButtonStyleREL;
lv_style_t myButtonStylePR;
lv_style_t myButtonStyleREL2;
lv_style_t myButtonStylePR2;

int autonSelected = 0;
int count = 0;



void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
}

static lv_res_t btn_click_action5(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn);
    if(id == 0){autonSelected = 1;}
    return LV_RES_OK;
}

static lv_res_t btn_click_action6(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn);
    if(id == 0){autonSelected = 2;}
    return LV_RES_OK;
}

static lv_res_t btn_click_action7(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn);
    if(id == 0){autonSelected = 3;}
    return LV_RES_OK;
}

static lv_res_t btn_click_action8(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn);
    if(id == 0){autonSelected = 4;}
    return LV_RES_OK;
}

static lv_res_t btn_click_action9(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn);
    if(id == 0){autonSelected = 5;}
    return LV_RES_OK;
}

static lv_res_t btn_click_action10(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn);
    if(id == 0){autonSelected = 6;}
    return LV_RES_OK;
}

static lv_res_t btn_click_action11(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn);
    if(id == 0){autonSelected = 7;}
    return LV_RES_OK;
}

static lv_res_t btn_click_action12(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn);
    if(id == 0){autonSelected = 8;}
    return LV_RES_OK;
}

static lv_res_t btn_click_action13(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn);
    if(id == 0){autonSelected = 9;}
    return LV_RES_OK;
}

static lv_res_t btn_click_action14(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn);
    if(id == 0){autonSelected = 10;}
    return LV_RES_OK;
}

static lv_res_t btn_click_action15(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn);
    if(id == 0){autonSelected = 11;}
    return LV_RES_OK;
}

static lv_res_t btn_click_action16(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn);
    if(id == 0){autonSelected = 12;}
    return LV_RES_OK;
}

static lv_res_t btn_click_action17(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn);
    if(id == 0){autonSelected = 13;}
    return LV_RES_OK;
}

static lv_res_t btn_click_action18(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn);
    if(id == 0){autonSelected = 14;}
    return LV_RES_OK;
}

static lv_res_t btn_click_action19(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn);
    if(id == 0){autonSelected = 15;}
    return LV_RES_OK;
}

static lv_res_t btn_click_action20(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn);
    if(id == 0){autonSelected = 16;}
    return LV_RES_OK;
}


static lv_res_t btn_click_action(lv_obj_t * btn) //Red Back (w/ Skills)
{
    uint8_t id = lv_obj_get_free_num(btn);

    if(id == 0)
    {
        lv_obj_clean(lv_scr_act());
        myButton = lv_btn_create(lv_scr_act(), NULL); 
        lv_obj_set_free_num(myButton, 0); 
        lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action16); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_REL, &myButtonStyleREL); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_PR, &myButtonStylePR); 
        lv_obj_set_size(myButton, 225, 105); 
        lv_obj_align(myButton, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 10); 

        myButtonLabel = lv_label_create(myButton, NULL); 
        lv_label_set_text(myButtonLabel, "Five (5)"); 

        myButton = lv_btn_create(lv_scr_act(), NULL); 
        lv_obj_set_free_num(myButton, 0); 
        lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action15); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_REL, &myButtonStyleREL); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_PR, &myButtonStylePR); 
        lv_obj_set_size(myButton, 225, 105); 
        lv_obj_align(myButton, NULL, LV_ALIGN_IN_TOP_RIGHT, -10, 10); 

        myButtonLabel = lv_label_create(myButton, NULL); 
        lv_label_set_text(myButtonLabel, "Six (6)"); 

        myButton = lv_btn_create(lv_scr_act(), NULL); 
        lv_obj_set_free_num(myButton, 0); 
        lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action14); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_REL, &myButtonStyleREL); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_PR, &myButtonStylePR); 
        lv_obj_set_size(myButton, 225, 105); 
        lv_obj_align(myButton, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 10, -10); 

        myButtonLabel = lv_label_create(myButton, NULL); 
        lv_label_set_text(myButtonLabel, "Seven (7)"); 

        myButton = lv_btn_create(lv_scr_act(), NULL); 
        lv_obj_set_free_num(myButton, 0); 
        lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action19); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_REL, &myButtonStyleREL); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_PR, &myButtonStylePR); 
        lv_obj_set_size(myButton, 225, 105); 
        lv_obj_align(myButton, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -10, -10); 

        myButtonLabel = lv_label_create(myButton, NULL); 
        lv_label_set_text(myButtonLabel, "Skills"); 
    }

    return LV_RES_OK;
}

static lv_res_t btn_click_action2(lv_obj_t * btn) //Blue Back
{
    uint8_t id = lv_obj_get_free_num(btn);

    if(id == 0)
    {
        lv_obj_clean(lv_scr_act());
        myButton = lv_btn_create(lv_scr_act(), NULL); 
        lv_obj_set_free_num(myButton, 0); 
        lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action8); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_REL, &myButtonStyleREL2); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_PR, &myButtonStylePR2); 
        lv_obj_set_size(myButton, 225, 105); 
        lv_obj_align(myButton, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 10); 

        myButtonLabel = lv_label_create(myButton, NULL); 
        lv_label_set_text(myButtonLabel, "Five (5)"); 

        myButton = lv_btn_create(lv_scr_act(), NULL); 
        lv_obj_set_free_num(myButton, 0); 
        lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action7); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_REL, &myButtonStyleREL2); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_PR, &myButtonStylePR2); 
        lv_obj_set_size(myButton, 225, 105); 
        lv_obj_align(myButton, NULL, LV_ALIGN_IN_TOP_RIGHT, -10, 10); 

        myButtonLabel = lv_label_create(myButton, NULL); 
        lv_label_set_text(myButtonLabel, "Six (6)"); 

        myButton = lv_btn_create(lv_scr_act(), NULL); 
        lv_obj_set_free_num(myButton, 0); 
        lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action6); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_REL, &myButtonStyleREL2); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_PR, &myButtonStylePR2); 
        lv_obj_set_size(myButton, 225, 105); 
        lv_obj_align(myButton, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 10, -10); 

        myButtonLabel = lv_label_create(myButton, NULL); 
        lv_label_set_text(myButtonLabel, "Seven (7)"); 

        myButton = lv_btn_create(lv_scr_act(), NULL); 
        lv_obj_set_free_num(myButton, 0); 
        lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action5); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_REL, &myButtonStyleREL2); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_PR, &myButtonStylePR2); 
        lv_obj_set_size(myButton, 225, 105); 
        lv_obj_align(myButton, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -10, -10); 

        myButtonLabel = lv_label_create(myButton, NULL); 
        lv_label_set_text(myButtonLabel, "Push"); 
    }

    return LV_RES_OK;
}

static lv_res_t btn_click_action3(lv_obj_t * btn) //Red Front
{
    uint8_t id = lv_obj_get_free_num(btn);

    if(id == 0)
    {
        lv_obj_clean(lv_scr_act());
        myButton = lv_btn_create(lv_scr_act(), NULL); 
        lv_obj_set_free_num(myButton, 0); 
        lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action17); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_REL, &myButtonStyleREL); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_PR, &myButtonStylePR); 
        lv_obj_set_size(myButton, 225, 105); 
        lv_obj_align(myButton, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 10); 

        myButtonLabel = lv_label_create(myButton, NULL); 
        lv_label_set_text(myButtonLabel, "Three (3)"); 

        myButton = lv_btn_create(lv_scr_act(), NULL); 
        lv_obj_set_free_num(myButton, 0); 
        lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action18); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_REL, &myButtonStyleREL); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_PR, &myButtonStylePR); 
        lv_obj_set_size(myButton, 225, 105); 
        lv_obj_align(myButton, NULL, LV_ALIGN_IN_TOP_RIGHT, -10, 10); 

        myButtonLabel = lv_label_create(myButton, NULL); 
        lv_label_set_text(myButtonLabel, "Four (4)"); 

        myButton = lv_btn_create(lv_scr_act(), NULL); 
        lv_obj_set_free_num(myButton, 0); 
        lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action5); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_REL, &myButtonStyleREL); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_PR, &myButtonStylePR); 
        lv_obj_set_size(myButton, 225, 105); 
        lv_obj_align(myButton, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 10, -10); 

        myButtonLabel = lv_label_create(myButton, NULL); 
        lv_label_set_text(myButtonLabel, "Push"); 

        myButton = lv_btn_create(lv_scr_act(), NULL); 
        lv_obj_set_free_num(myButton, 0); 
        lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action5); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_REL, &myButtonStyleREL); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_PR, &myButtonStylePR); 
        lv_obj_set_size(myButton, 225, 105); 
        lv_obj_align(myButton, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -10, -10); 

        myButtonLabel = lv_label_create(myButton, NULL); 
        lv_label_set_text(myButtonLabel, "Push"); 
    }

    return LV_RES_OK;
}

static lv_res_t btn_click_action4(lv_obj_t * btn) //Blue Front
{
    uint8_t id = lv_obj_get_free_num(btn);

    if(id == 0)
    {
        lv_obj_clean(lv_scr_act());
        myButton = lv_btn_create(lv_scr_act(), NULL); 
        lv_obj_set_free_num(myButton, 0); 
        lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action9); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_REL, &myButtonStyleREL2); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_PR, &myButtonStylePR2); 
        lv_obj_set_size(myButton, 225, 105); 
        lv_obj_align(myButton, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 10); 

        myButtonLabel = lv_label_create(myButton, NULL); 
        lv_label_set_text(myButtonLabel, "Three (3)"); 

        myButton = lv_btn_create(lv_scr_act(), NULL); 
        lv_obj_set_free_num(myButton, 0); 
        lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action10); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_REL, &myButtonStyleREL2); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_PR, &myButtonStylePR2); 
        lv_obj_set_size(myButton, 225, 105); 
        lv_obj_align(myButton, NULL, LV_ALIGN_IN_TOP_RIGHT, -10, 10); 

        myButtonLabel = lv_label_create(myButton, NULL); 
        lv_label_set_text(myButtonLabel, "Four (4)"); 

        myButton = lv_btn_create(lv_scr_act(), NULL); 
        lv_obj_set_free_num(myButton, 0); 
        lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action5); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_REL, &myButtonStyleREL2); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_PR, &myButtonStylePR2); 
        lv_obj_set_size(myButton, 225, 105); 
        lv_obj_align(myButton, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 10, -10); 

        myButtonLabel = lv_label_create(myButton, NULL); 
        lv_label_set_text(myButtonLabel, "Push"); 

        myButton = lv_btn_create(lv_scr_act(), NULL); 
        lv_obj_set_free_num(myButton, 0); 
        lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action5); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_REL, &myButtonStyleREL2); 
        lv_btn_set_style(myButton, LV_BTN_STYLE_PR, &myButtonStylePR2); 
        lv_obj_set_size(myButton, 225, 105); 
        lv_obj_align(myButton, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -10, -10); 

        myButtonLabel = lv_label_create(myButton, NULL); 
        lv_label_set_text(myButtonLabel, "Push"); 
    }

    return LV_RES_OK;
}

void initialize() {
    //autonSelected = 1;
    delay(20);

    lv_style_copy(&myButtonStyleREL, &lv_style_plain);
    myButtonStyleREL.body.main_color = LV_COLOR_MAKE(150, 0, 0);
    myButtonStyleREL.body.grad_color = LV_COLOR_MAKE(200, 200, 200);
    myButtonStyleREL.body.radius = 10;
    myButtonStyleREL.text.color = LV_COLOR_MAKE(255, 255, 255);

    lv_style_copy(&myButtonStylePR, &lv_style_plain);
    myButtonStylePR.body.main_color = LV_COLOR_MAKE(255, 0, 0);
    myButtonStylePR.body.grad_color = LV_COLOR_MAKE(255, 255, 255);
    myButtonStylePR.body.radius = 10;
    myButtonStylePR.text.color = LV_COLOR_MAKE(255, 255, 255);

    myButton = lv_btn_create(lv_scr_act(), NULL); 
    lv_obj_set_free_num(myButton, 0); 
    lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action3); 
    lv_btn_set_style(myButton, LV_BTN_STYLE_REL, &myButtonStyleREL); 
    lv_btn_set_style(myButton, LV_BTN_STYLE_PR, &myButtonStylePR); 
    lv_obj_set_size(myButton, 225, 105); 
    lv_obj_align(myButton, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 10); 

    myButtonLabel = lv_label_create(myButton, NULL); 
    lv_label_set_text(myButtonLabel, "Red Front"); 

    myButton = lv_btn_create(lv_scr_act(), NULL); 
    lv_obj_set_free_num(myButton, 0); 
    lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action); 
    lv_btn_set_style(myButton, LV_BTN_STYLE_REL, &myButtonStyleREL); 
    lv_btn_set_style(myButton, LV_BTN_STYLE_PR, &myButtonStylePR); 
    lv_obj_set_size(myButton, 225, 105); 
    lv_obj_align(myButton, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 10, -10); 

    myButtonLabel = lv_label_create(myButton, NULL); 
    lv_label_set_text(myButtonLabel, "Red Back"); 

    lv_style_copy(&myButtonStyleREL2, &lv_style_plain);
    myButtonStyleREL2.body.main_color = LV_COLOR_MAKE(0, 0, 150);
    myButtonStyleREL2.body.grad_color = LV_COLOR_MAKE(200, 200, 200);
    myButtonStyleREL2.body.radius = 10;
    myButtonStyleREL2.text.color = LV_COLOR_MAKE(255, 255, 255);

    lv_style_copy(&myButtonStylePR2, &lv_style_plain);
    myButtonStylePR2.body.main_color = LV_COLOR_MAKE(0, 0, 255);
    myButtonStylePR2.body.grad_color = LV_COLOR_MAKE(255, 255, 255);
    myButtonStylePR2.body.radius = 10;
    myButtonStylePR2.text.color = LV_COLOR_MAKE(255, 255, 255);

    myButton = lv_btn_create(lv_scr_act(), NULL); 
    lv_obj_set_free_num(myButton, 0); 
    lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action4); 
    lv_btn_set_style(myButton, LV_BTN_STYLE_REL, &myButtonStyleREL2); 
    lv_btn_set_style(myButton, LV_BTN_STYLE_PR, &myButtonStylePR2); 
    lv_obj_set_size(myButton, 225, 105); 
    lv_obj_align(myButton, NULL, LV_ALIGN_IN_TOP_RIGHT, -10, 10); 

    myButtonLabel = lv_label_create(myButton, NULL); 
    lv_label_set_text(myButtonLabel, "Blue Front"); 

    myButton = lv_btn_create(lv_scr_act(), NULL); 
    lv_obj_set_free_num(myButton, 0); 
    lv_btn_set_action(myButton, LV_BTN_ACTION_CLICK, btn_click_action2); 
    lv_btn_set_style(myButton, LV_BTN_STYLE_REL, &myButtonStyleREL2); 
    lv_btn_set_style(myButton, LV_BTN_STYLE_PR, &myButtonStylePR2); 
    lv_obj_set_size(myButton, 225, 105); 
    lv_obj_align(myButton, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -10, -10); 

    myButtonLabel = lv_label_create(myButton, NULL); 
    lv_label_set_text(myButtonLabel, "Blue Back"); 



    //lv_obj_t * logo1 = lv_img_create(lv_scr_act(), NULL); /*Crate an image object*/
    //lv_img_set_src(logo1, &logo1);  /*Set the created file as image (a red fl  ower)*/
    //lv_obj_set_pos(logo1, 10, 10);      /*Set the positions*/
    //lv_obj_set_drag(logo1, true);
}

/*void initialize() {

    lv_obj_t * logo1 = lv_img_create(lv_scr_act(), NULL); /*Crate an image object*/
    //lv_img_set_src(logo1, &logo1);  /*Set the created file as image (a red fl  ower)*/
    //lv_obj_set_pos(logo1, 10, 10);      /*Set the positions*/
    //lv_obj_set_drag(logo1, true);
    //lcd::register_btn1_cb(on_center_button);


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}