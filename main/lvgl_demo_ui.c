/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// This demo UI is adapted from LVGL official example: https://docs.lvgl.io/master/widgets/extra/meter.html#simple-meter

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "lvgl.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/ledc.h"

static const char *TAG = "APP";

LV_FONT_DECLARE(segoe_ui_15);
LV_FONT_DECLARE(segoe_ui_80);
LV_FONT_DECLARE(segoe_ui_30);
LV_FONT_DECLARE(custom_sym);

#define WORK_SYM "\xEF\x82\xB1"
#define PRAY_SYM "\xEF\x9A\x83"

#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE

#define ID_CW       4
#define LEDC_LS_CH1_CHANNEL    LEDC_CHANNEL_1

#define ID_WW       5
#define LEDC_LS_CH2_CHANNEL    LEDC_CHANNEL_2

#define D_CW       10
#define LEDC_LS_CH3_CHANNEL    LEDC_CHANNEL_3

#define D_WW       11
#define LEDC_LS_CH4_CHANNEL    LEDC_CHANNEL_4


int settings[8] = {0};

typedef enum
{
    work_direct_cw = 0,
    work_direct_ww,
    work_indirect_cw,
    work_indirect_ww,
    med_direct_cw,
    med_direct_ww,
    med_indirect_cw,
    med_indirect_ww,
} control_state_t;

typedef enum
{
    WORK_MODE = 0,
    MEDITATION_MODE,
}noor_mode_t;

lv_obj_t *slider;
lv_obj_t *btnDirect;
lv_obj_t *btnIndirect;
lv_obj_t *sw;
noor_mode_t noorMode = WORK_MODE;

control_state_t controlSelected = work_direct_cw;

/*
void delete_popup_message(lv_obj_t ** mbox)
{
    lv_msgbox_close(*mbox);
}

void show_popup_message()
{
    char note[50] = {0};
    switch(controlSelected)
    {
        case work_direct_cw: strcpy(note, "[work]: direct cold white");break;
        case work_direct_ww: strcpy(note, "[work]: direct warm white");break;
        case work_indirect_cw: strcpy(note, "[work]: indirect cold white");break;
        case work_indirect_ww: strcpy(note, "[work]: indirect warm white");break;
        case med_direct_cw: strcpy(note, "[meditation]: direct cold white");break;
        case med_direct_ww: strcpy(note, "[meditation]: direct warm white");break;
        case med_indirect_cw: strcpy(note, "[meditation]: indirect cold white");break;
        case med_indirect_ww: strcpy(note, "[meditation]: indirect warm white");break;
    }
    lv_obj_t * mbox = lv_msgbox_create(lv_scr_act(), "In Control", note, NULL, false);
    lv_obj_center(mbox);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, mbox);
    lv_anim_set_time(&a, 1200);
    lv_anim_set_delay(&a, 0);
    lv_anim_set_deleted_cb(&a, delete_popup_message);
    lv_anim_start(&a);
}
*/

void driver_init(void)
{
    int ch;
    /*1- Timer Configuration by specifying the PWM signal’s frequency and duty cycle resolution.*/
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        .speed_mode = LEDC_LS_MODE,           // timer mode
        .timer_num = LEDC_LS_TIMER,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    /*2- Channel Configuration by associating it with the timer and GPIO to output the PWM signal.*/
        ledc_channel_config_t ledc_channel[4] = {
        {
            .channel    = LEDC_LS_CH1_CHANNEL,
            .duty       = 0,
            .gpio_num   = ID_CW,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER,
            .flags.output_invert = 0
        },
        {
            .channel    = LEDC_LS_CH2_CHANNEL,
            .duty       = 0,
            .gpio_num   = ID_WW,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER,
            .flags.output_invert = 0
        },
        {
            .channel    = LEDC_LS_CH3_CHANNEL,
            .duty       = 0,
            .gpio_num   = D_CW,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER,
            .flags.output_invert = 0
        },
        {
            .channel    = LEDC_LS_CH4_CHANNEL,
            .duty       = 0,
            .gpio_num   = D_WW,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER,
            .flags.output_invert = 0
        },
    };
    for (ch = 0; ch < 4; ch++) {
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel[ch]));
    }
}


static void set_mode_work(void) 
{
    ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH3_CHANNEL, (settings[work_direct_cw] * 900) / 100);
    ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH3_CHANNEL);

    ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH4_CHANNEL, (settings[work_direct_cw] * 900) / 100);
    ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH4_CHANNEL);

    ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH1_CHANNEL, (settings[work_indirect_cw] * 900) / 100);
    ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH1_CHANNEL);

    ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH2_CHANNEL, (settings[work_indirect_ww] * 900) / 100);
    ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH2_CHANNEL);
}


static void set_mode_meditation(void) 
{
    ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH3_CHANNEL, (settings[med_direct_cw] * 900) / 100);
    ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH3_CHANNEL);

    ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH4_CHANNEL, (settings[med_direct_cw] * 900) / 100);
    ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH4_CHANNEL);

    ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH1_CHANNEL, (settings[med_indirect_cw] * 900) / 100);
    ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH1_CHANNEL);

    ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH2_CHANNEL, (settings[med_indirect_ww] * 900) / 100);
    ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH2_CHANNEL);
}

static void handler_selectors_mode(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    
    if(code == LV_EVENT_VALUE_CHANGED && obj == sw)
    {
        if(lv_obj_has_state(obj, LV_STATE_CHECKED))
        {
            noorMode = MEDITATION_MODE;
            controlSelected = med_direct_cw;
            lv_slider_set_value(slider, settings[med_direct_cw], LV_ANIM_ON);
            set_mode_meditation();

        }
        else
        {
            noorMode = WORK_MODE;
            controlSelected = work_direct_cw;
            lv_slider_set_value(slider, settings[work_direct_cw], LV_ANIM_ON);
            set_mode_work();
        }
    }
}

static void handler_selectors_click(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);

    if(code == LV_EVENT_SHORT_CLICKED)
    {
        lv_color_t current_color = lv_obj_get_style_bg_color(obj, LV_PART_MAIN);

        // Toggle zwischen btnDirect und btnIndirect Design
        if (current_color.full == lv_color_hex(0xB96E48).full) { // Direktes Design
            // Wechseln zu Indirektem Design
            lv_obj_set_style_bg_color(obj, lv_color_hex(0x8DB7C7), 0);
            lv_obj_set_style_shadow_color(obj, lv_color_hex(0x535E63), 0);
        } else {
            // Wechseln zu Direktem Design
            lv_obj_set_style_bg_color(obj, lv_color_hex(0xB96E48), 0);
            lv_obj_set_style_shadow_color(obj, lv_color_hex(0x642917), 0);
        }

        ESP_LOGI(TAG, "click");
        if (obj == btnDirect)
        {

            if(noorMode == WORK_MODE)
            {
                // Umschalten zwischen work_direct_cw und work_direct_ww
                if(controlSelected == work_direct_cw)
                {
                    controlSelected = work_direct_ww;
                    lv_slider_set_value(slider, settings[work_direct_ww], LV_ANIM_OFF);
                }
                else
                {
                    controlSelected = work_direct_cw;
                    lv_slider_set_value(slider, settings[work_direct_cw], LV_ANIM_OFF);
                }
            }
            else if (noorMode == MEDITATION_MODE)
            {
                // Umschalten zwischen med_direct_cw und med_direct_ww
                if(controlSelected == med_direct_cw)
                {
                    controlSelected = med_direct_ww;
                    lv_slider_set_value(slider, settings[med_direct_ww], LV_ANIM_OFF);
                }
                else
                {
                    controlSelected = med_direct_cw;
                    lv_slider_set_value(slider, settings[med_direct_cw], LV_ANIM_OFF);
                }
            }
            else
                ESP_LOGI(TAG, "unknown mode");
        }
        else if (obj == btnIndirect)
            {
                if(noorMode == WORK_MODE)
                {
                    if(controlSelected == work_indirect_cw)
                    {
                        controlSelected = work_indirect_ww;
                        lv_slider_set_value(slider, settings[work_indirect_ww], LV_ANIM_OFF);
                    }
                    else
                    {
                        controlSelected = work_indirect_cw;
                        lv_slider_set_value(slider, settings[work_indirect_cw], LV_ANIM_OFF);
                    }
                }
                else if(noorMode == MEDITATION_MODE)
                {
                    if(controlSelected == med_indirect_cw)
                    {
                        controlSelected = med_indirect_ww;
                        lv_slider_set_value(slider, settings[med_indirect_ww], LV_ANIM_OFF);
                    }
                    else
                    {
                        controlSelected = med_indirect_cw;
                        lv_slider_set_value(slider, settings[med_indirect_cw], LV_ANIM_OFF);
                    }

                }
                else
                    ESP_LOGI(TAG, "unknown mode");
            }
        else
                ESP_LOGI(TAG, "unknown obj");
    }

}
static void handler_selectors_longpress(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_LONG_PRESSED)
    {
    ESP_LOGI(TAG, "Long Pressed");
    if (obj == btnDirect)
        {
            if(noorMode == WORK_MODE)
            {
                    controlSelected = work_direct_ww;
                    lv_slider_set_value(slider, settings[work_direct_ww], LV_ANIM_OFF);
            }
            else if (noorMode == MEDITATION_MODE)
            {
                    controlSelected = med_direct_cw;
                    lv_slider_set_value(slider, settings[med_direct_ww], LV_ANIM_OFF);
            }
            else
                ESP_LOGI(TAG, "unknown mode");
        }

        else if (obj == btnIndirect)
            {
                if(noorMode == WORK_MODE)
                {
                    controlSelected = work_indirect_ww;
                    lv_slider_set_value(slider, settings[work_indirect_ww], LV_ANIM_OFF);
                }
                else if(noorMode == MEDITATION_MODE)
                {
                    controlSelected = med_indirect_cw;
                    lv_slider_set_value(slider, settings[med_indirect_cw], LV_ANIM_OFF);
                }
                else
                    ESP_LOGI(TAG, "unknown mode");
            }
        else
                ESP_LOGI(TAG, "unknown obj");
    }
    //show_popup_message();
    ESP_LOGI(TAG, "mode : control (%d:%d)",noorMode, controlSelected);
}

static void slider_event_cb(lv_event_t * e)
{
    uint16_t g_value = (lv_slider_get_value(slider) * 900) / 100;

    /*control the PWM signal*/
    switch(controlSelected)
    {
        case work_direct_cw:   ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH3_CHANNEL, g_value); ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH3_CHANNEL);
                               settings[work_direct_cw] = lv_slider_get_value(slider); break;
        case work_direct_ww:   ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH4_CHANNEL, g_value); ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH4_CHANNEL);    
                               settings[work_direct_ww] = lv_slider_get_value(slider); break;
        case work_indirect_cw: ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH1_CHANNEL, g_value); ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH1_CHANNEL);
                               settings[work_indirect_cw] = lv_slider_get_value(slider); break;
        case work_indirect_ww: ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH2_CHANNEL, g_value); ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH2_CHANNEL);
                               settings[work_indirect_ww] = lv_slider_get_value(slider); break;
        case med_direct_cw:    ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH3_CHANNEL, g_value); ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH3_CHANNEL);
                               settings[med_direct_cw] = lv_slider_get_value(slider); break;
        case med_direct_ww:    ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH4_CHANNEL, g_value); ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH4_CHANNEL);
                               settings[med_direct_ww] = lv_slider_get_value(slider); break;
        case med_indirect_cw:  ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH1_CHANNEL, g_value); ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH1_CHANNEL);
                               settings[med_indirect_cw] = lv_slider_get_value(slider); break;
        case med_indirect_ww:  ledc_set_duty(LEDC_LS_MODE, LEDC_LS_CH2_CHANNEL, g_value); ledc_update_duty(LEDC_LS_MODE, LEDC_LS_CH2_CHANNEL);
                               settings[med_indirect_ww] = lv_slider_get_value(slider); break;
    }

    ESP_LOGI(TAG, "value %d to %d",g_value, controlSelected);
}

// // function to apply of the styles
// void apply_warm_white_style(lv_obj_t *btn) {
//     lv_obj_set_style_bg_color(btn, lv_color_hex(0xB96E48), 0); // warm white style
//     lv_obj_set_style_shadow_color(btn, lv_color_hex(0x642917), 0);
// }

// void apply_cold_white_style(lv_obj_t *btn) {
//     lv_obj_set_style_bg_color(btn, lv_color_hex(0x8DB7C7), 0); // cold white style
//     lv_obj_set_style_shadow_color(btn, lv_color_hex(0x535E63), 0);
// }


void LightControlUI(void)
{
    // General settings
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);

    // 1- container of buttons
    lv_obj_t * btnsContainer = lv_obj_create(lv_scr_act());
    lv_obj_set_size(btnsContainer, 150, 240);
    lv_obj_set_style_border_width(btnsContainer, 0, 0);
    lv_obj_set_style_bg_color(btnsContainer, lv_color_black(), 0);
    lv_obj_set_x(btnsContainer, 0);
    lv_obj_set_y(btnsContainer, 0);
    lv_obj_clear_flag(btnsContainer, LV_OBJ_FLAG_SCROLLABLE);

    // 1- 1 button indirect
    btnIndirect = lv_btn_create(btnsContainer);
    lv_obj_set_size(btnIndirect, 75, 75);
    lv_obj_set_style_radius(btnIndirect, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_align(btnIndirect, LV_ALIGN_TOP_MID, 0, 20);
    lv_obj_set_style_bg_color(btnIndirect,  lv_color_hex(0x8DB7C7), 0);
    lv_obj_set_style_shadow_color(btnIndirect, lv_color_hex(0x535E63), 0);
    lv_obj_set_style_shadow_opa(btnIndirect, LV_OPA_COVER, 0);
    lv_obj_set_style_shadow_width(btnIndirect, 12, 0);
    lv_obj_set_style_shadow_spread(btnIndirect, 12, 0);
    //lv_obj_add_event_cb(btnIndirect, handler_selectors_longpress, LV_EVENT_LONG_PRESSED , NULL);
    lv_obj_add_event_cb(btnIndirect, handler_selectors_click, LV_EVENT_SHORT_CLICKED, NULL);

    // 1- 2 label indirect
    lv_obj_t * textBtnIndirect = lv_label_create(btnsContainer);
    lv_obj_set_style_text_color(textBtnIndirect, lv_color_white(), 0);
    lv_label_set_text(textBtnIndirect, "INDIRECT");
    lv_obj_align_to(textBtnIndirect, btnIndirect, LV_ALIGN_OUT_TOP_MID, 0, -12);

    //1- 3 button direct
    btnDirect = lv_btn_create(btnsContainer);
    lv_obj_set_size(btnDirect, 75, 75);
    lv_obj_set_style_radius(btnDirect, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_align(btnDirect, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(btnDirect,  lv_color_hex(0xB96E48), 0);
    lv_obj_set_style_shadow_color(btnDirect, lv_color_hex(0x642917), 0);
    lv_obj_set_style_shadow_opa(btnDirect, LV_OPA_COVER, 0);
    lv_obj_set_style_shadow_width(btnDirect, 12, 0);
    lv_obj_set_style_shadow_spread(btnDirect, 12, 0);
    // lv_obj_add_event_cb(btnDirect, handler_selectors_longpress, LV_EVENT_LONG_PRESSED , NULL);
    lv_obj_add_event_cb(btnDirect, handler_selectors_click, LV_EVENT_SHORT_CLICKED , NULL);

    // 1- 4 label direct
    lv_obj_t * textBtnDirect = lv_label_create(btnsContainer);
    lv_obj_set_style_text_color(textBtnDirect, lv_color_white(), 0);
    lv_label_set_text(textBtnDirect, "DIRECT");
    lv_obj_align_to(textBtnDirect, btnDirect, LV_ALIGN_OUT_TOP_MID, 0,  -12);

    // 2- container of slider
    lv_obj_t * sliderContainer = lv_obj_create(lv_scr_act());
    lv_obj_set_size(sliderContainer, 90, 240);
    lv_obj_set_style_border_width(sliderContainer, 0, 0);
    lv_obj_set_style_bg_color(sliderContainer, lv_color_black(), 0);
    lv_obj_set_x(sliderContainer, 150);
    lv_obj_set_y(sliderContainer, 0);
    lv_obj_clear_flag(sliderContainer, LV_OBJ_FLAG_SCROLLABLE);

    //2- 1 slider
    slider = lv_slider_create(sliderContainer);
    lv_obj_set_size(slider, 70, 180);

    static lv_style_t style_slider;
    static lv_style_t style_knob;

    lv_style_init(&style_slider);
    lv_style_set_bg_opa(&style_slider, LV_OPA_COVER);
    lv_style_set_bg_color(&style_slider, lv_color_white());
    lv_style_set_bg_grad_color(&style_slider, lv_color_black());
    lv_style_set_bg_grad_dir(&style_slider, LV_GRAD_DIR_VER);
    lv_style_set_bg_grad_stop(&style_slider, 250);
    lv_style_set_radius(&style_slider, LV_RADIUS_CIRCLE);

    lv_style_init(&style_knob);
    lv_style_set_bg_opa(&style_knob, LV_OPA_COVER);
    lv_style_set_bg_color(&style_knob, lv_color_hex(0xD9D9D9));
    lv_style_set_border_width(&style_knob, 0);
    lv_style_set_pad_all(&style_knob, 0);

    lv_obj_center(slider);
    lv_obj_add_style(slider, &style_slider, LV_PART_MAIN);
    lv_obj_add_style(slider, &style_slider, LV_PART_INDICATOR);
    lv_obj_add_style(slider, &style_knob, LV_PART_KNOB);
    lv_obj_add_event_cb(slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    //3- modes
    //3- 1- modes container
    lv_obj_t * modesContainer = lv_obj_create(lv_scr_act());
    lv_obj_set_size(modesContainer, 240, 40);
    lv_obj_set_style_border_width(modesContainer, 0, 0);
    lv_obj_set_style_bg_color(modesContainer, lv_color_black(), 0);
    lv_obj_set_x(modesContainer, 0);
    lv_obj_set_y(modesContainer, 240);
    lv_obj_clear_flag(modesContainer, LV_OBJ_FLAG_SCROLLABLE);

    //3- 2- switch
    sw = lv_switch_create(modesContainer);
    lv_obj_set_style_bg_color(sw, lv_color_hex(0xB96E48), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(sw, lv_color_hex(0x8DB7C7), 0);
    lv_obj_add_event_cb(sw, handler_selectors_mode, LV_EVENT_VALUE_CHANGED , NULL);
    lv_obj_set_size(sw, 80, 35);
    lv_obj_center(sw);

    //3- 3- work icon
    lv_obj_t * workIcon = lv_label_create(modesContainer);
    lv_obj_set_style_text_font(workIcon, &custom_sym, 0);
    lv_obj_set_style_text_color(workIcon, lv_color_hex(0x8DB7C7), 0);
    lv_label_set_text(workIcon, WORK_SYM);
    lv_obj_align_to(workIcon, sw, LV_ALIGN_OUT_LEFT_MID, -10, 0);

    //3- 4- meditate icon
    lv_obj_t * medIcon = lv_label_create(modesContainer);
    lv_obj_set_style_text_font(medIcon, &custom_sym, 0);
    lv_obj_set_style_text_color(medIcon, lv_color_hex(0xB96E48), 0);
    lv_label_set_text(medIcon, PRAY_SYM);
    lv_obj_align_to(medIcon, sw, LV_ALIGN_OUT_RIGHT_MID, 10, 0);
    /**/
    }
