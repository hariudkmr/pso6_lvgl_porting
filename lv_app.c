/*
 * lv_app.c
 *
 *  Created on: 25-May-2023
 *      Author: udayakumar
 */

#include "lvgl.h"

lv_obj_t *btn1;
lv_obj_t *btn2;
lv_obj_t *screenMain;
lv_obj_t *label;

static void btn_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);
    if(code == LV_EVENT_PRESSED) {
        static uint8_t cnt = 0;
        cnt++;

        /*Get the first child of the button which is the label and change its text*/
        lv_obj_t * label = lv_obj_get_child(btn, 0);
        lv_label_set_text_fmt(label, "Pressed: %d",cnt);
    }
}

void app_main()
{
	/*Create screen objects*/
    /*Change the active screen's background color*/
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x003a57), LV_PART_MAIN);

    /*Create a white label, set its text and align it to the center*/
    lv_obj_t * label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Hello world");
    lv_obj_set_style_text_color(lv_scr_act(), lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t * btn1 = lv_btn_create(lv_scr_act());     /*Add a button the current screen*/
	lv_obj_set_pos(btn1, 10, 10);                            /*Set its position*/
	lv_obj_set_size(btn1, 120, 50);                          /*Set its size*/
	lv_obj_add_event(btn1, btn_event_cb, LV_EVENT_ALL, NULL);           /*Assign a callback to the button*/

	lv_obj_t * label1 = lv_label_create(btn1);          /*Add a label to the button*/
	lv_label_set_text(label1, "Button1:");                     /*Set the labels text*/
	lv_obj_center(label1);

	lv_obj_t * btn2 = lv_btn_create(lv_scr_act());     /*Add a button the current screen*/
	lv_obj_set_pos(btn2, 150, 160);                            /*Set its position*/
	lv_obj_set_size(btn2, 120, 50);                          /*Set its size*/
	lv_obj_add_event(btn2, btn_event_cb, LV_EVENT_ALL, NULL);           /*Assign a callback to the button*/

	lv_obj_t * label2 = lv_label_create(btn2);          /*Add a label to the button*/
	lv_label_set_text(label2, "Button2:");                     /*Set the labels text*/
	lv_obj_center(label2);

}



