#if __has_include(<Arduino.h>)
#include <Arduino.h>
#else
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <button.hpp>
#include <lvgl.h>
// font is a TTF/OTF from downloaded from fontsquirrel.com
// converted to a header with https://honeythecodewitch.com/gfx/converter
#define OPENSANS_REGULAR_IMPLEMENTATION
#include <assets/OpenSans_Regular.hpp>

// namespace imports
#ifdef ARDUINO
using namespace arduino;
#else
using namespace esp_idf;
#endif
// use two 32KB buffers (DMA)
static uint32_t lcd_transfer_buffer1[240*14*2/4];
static uint32_t lcd_transfer_buffer2[240*14*2/4];
// this is the handle from the esp panel api
static esp_lcd_panel_handle_t lcd_handle;
static lv_display_t* lcd_display = nullptr;

using button_t = multi_button;
static basic_button button_a_raw(35, 10, true);
static basic_button button_b_raw(0, 10, true);
button_t button_a(button_a_raw);
button_t button_b(button_b_raw);


// tell UIX the DMA transfer is complete
static bool lcd_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    if(lcd_display!=nullptr) {
        lv_display_flush_ready(lcd_display);
    }
    return true;
}
void lcd_flush_display( lv_display_t *disp, const lv_area_t *area, uint8_t * px_map) {
    size_t count = (area->x2-area->x1+1)*(area->y2-area->y1+1);
    for(int i = 0;i<count;++i) {
        uint16_t* p = &((uint16_t*)px_map)[i];
        *p = ((*p<<8)&0xFF00)|((*p>>8)&0xFF);
    }
    esp_lcd_panel_draw_bitmap(lcd_handle,area->x1,area->y1,area->x2+1,area->y2+1,px_map);
    LV_UNUSED(disp);
}
// initialize the screen using the esp panel API
static void lcd_panel_init()
{
    // backlight
    gpio_set_direction((gpio_num_t)4, GPIO_MODE_OUTPUT);
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.sclk_io_num = 18;
    buscfg.mosi_io_num = 19;
    buscfg.miso_io_num = -1;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = sizeof(lcd_transfer_buffer1) + 8;

    // Initialize the SPI bus on VSPI (SPI3)
    spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config;
    memset(&io_config, 0, sizeof(io_config));
    io_config.dc_gpio_num = 16,
    io_config.cs_gpio_num = 5,
    io_config.pclk_hz = 20 * 1000 * 1000,
    io_config.lcd_cmd_bits = 8,
    io_config.lcd_param_bits = 8,
    io_config.spi_mode = 0,
    io_config.trans_queue_depth = 10,
    io_config.on_color_trans_done = lcd_flush_ready;
    // Attach the LCD to the SPI bus
    esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &io_config, &io_handle);

    lcd_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config;
    memset(&panel_config, 0, sizeof(panel_config));
    panel_config.reset_gpio_num = 23;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    panel_config.rgb_endian = LCD_RGB_ENDIAN_RGB;
#else
    panel_config.color_space = ESP_LCD_COLOR_SPACE_RGB;
#endif
    panel_config.bits_per_pixel = 16;

    // Initialize the LCD configuration
    esp_lcd_new_panel_st7789(io_handle, &panel_config, &lcd_handle);

    // Turn off backlight to avoid unpredictable display on the LCD screen while initializing
    // the LCD panel driver. (Different LCD screens may need different levels)
    gpio_set_level((gpio_num_t)4, 0);
    // Reset the display
    esp_lcd_panel_reset(lcd_handle);

    // Initialize LCD panel
    esp_lcd_panel_init(lcd_handle);
    // esp_lcd_panel_io_tx_param(io_handle, LCD_CMD_SLPOUT, NULL, 0);
    //  Swap x and y axis (Different LCD screens may need different options)
    esp_lcd_panel_swap_xy(lcd_handle, true);
    esp_lcd_panel_set_gap(lcd_handle, 40, 52);
    esp_lcd_panel_mirror(lcd_handle, false, true);
    esp_lcd_panel_invert_color(lcd_handle, true);
    
    // Turn on the screen
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_lcd_panel_disp_on_off(lcd_handle, true);
#else
    esp_lcd_panel_disp_off(lcd_handle, false);
#endif
    // Turn on backlight (Different LCD screens may need different levels)
    gpio_set_level((gpio_num_t)4, 1);
}
static uint32_t lvgl_ticks() {
    return pdTICKS_TO_MS(xTaskGetTickCount());
}
void button_pressed(bool pressed, void* state) {
    
}

const lv_font_t* ui_text_font=nullptr;
static lv_style_t ui_text_font_style;
static lv_obj_t *ui_label ;
static void ui_init_font() {
    //lv_tiny_ttf_init();
    
    ui_text_font = lv_tiny_ttf_create_data(OpenSans_Regular_data,sizeof(OpenSans_Regular_data),70);
    if(ui_text_font==nullptr) {
        puts("Font initialization failure");
    }
    lv_style_init(&ui_text_font_style);
    lv_style_set_text_font(&ui_text_font_style,ui_text_font);
}
char banner[7] = {0};
size_t banner_length=0;
void ui_init() {
    ui_init_font();
    
    ui_label= lv_label_create(lv_screen_active());
    lv_obj_add_style(ui_label, &ui_text_font_style, 0); 
    lv_label_set_text( ui_label, "Hello from LVGL" );
    lv_obj_align( ui_label, LV_ALIGN_CENTER, 0, 0 );

    printf("Banner width is: %d pixels\n",(int)lv_text_get_width(banner,banner_length,ui_text_font,0));
    puts("Done");
}
#ifdef ARDUINO
void setup() {
    Serial.begin(115200);
#else
void loop();
extern "C" void app_main() {
#endif
    strcpy(banner,"hello!");
    banner_length = strlen(banner);
     
    lcd_panel_init();
    button_a.initialize();
    button_b.initialize();
    button_a.on_pressed_changed(button_pressed);
    button_b.on_pressed_changed(button_pressed);

    lv_init();
    lv_tick_set_cb(lvgl_ticks);
    lcd_display =lv_display_create(240, 135);
    lv_display_set_flush_cb(lcd_display, lcd_flush_display);
    lv_display_set_buffers(lcd_display, lcd_transfer_buffer1, lcd_transfer_buffer2, 240*14*2, LV_DISPLAY_RENDER_MODE_PARTIAL);
    /*Change the active screen's background color*/
    lv_obj_set_style_bg_color(lv_screen_active(),lv_color_make(0xFF,0xFF,0), LV_PART_MAIN);
    ui_init();
#ifndef ARDUINO
    while(1) {
        loop();
        static int count = 0;
        if(count++>6) {
            vTaskDelay(5);
            count = 0;
        }

    }
#endif
}
#ifndef ARDUINO
uint32_t millis() {
    return pdTICKS_TO_MS(xTaskGetTickCount());
}
#endif
void loop()
{
    static int iterations = 0;
    static uint32_t ips_ts = millis();
    if(millis()>=ips_ts+1000) {
        ips_ts = millis();
        printf("iterations: %d\n",iterations);
        iterations=0;
    }
    lv_label_set_text( ui_label, banner );
    char ch = banner[0];
    for(int i = 1;i<=banner_length;++i) {
        banner[i-1]=banner[i];
    }
    banner[banner_length-1]=ch;
    lv_timer_handler();
    ++iterations;
    //vTaskDelay(pdMS_TO_TICKS(100));
}
