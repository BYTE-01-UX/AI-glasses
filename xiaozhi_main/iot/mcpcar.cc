#include "board.h"
#include "mcp_server.h"
#include "driver/uart.h"
#include <driver/gpio.h>
#include <esp_log.h>
#include "mcpcar.h"

#define TAG "McpCar"
// 
// ML307    UART1  
#define UART1_TXD ML307_TX_PIN
#define UART1_RXD ML307_RX_PIN
#define BUF_SIZE (1024)

// void McpCar::uart1_init(void)
// {
//     uart_config_t uart_config = {
//         .baud_rate = 9600,
//         .data_bits = UART_DATA_8_BITS,
//         .parity = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//         .source_clk = UART_SCLK_APB,
//     };

//     uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
//     uart_set_pin(UART_NUM_1, UART1_TXD, UART1_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
//     uart_param_config(UART_NUM_1, &uart_config);
// }

// void McpCar::send_uart_command(uint8_t command)
// {
//     uart_write_bytes(UART_NUM_1, &command, 1);
// }

McpCar::McpCar()
{
    auto &mcp_server = McpServer::GetInstance(); // ① 定义引用

    mcp_server.AddTool("self.mcpcar.sit_down_relaxed",
                       "放松趴下",
                       PropertyList(), [this](const PropertyList &)
                       {
                           ESP_LOGI(TAG, "放松趴下");
                           send_uart_command(0x29);
                           return true; });

    mcp_server.AddTool("self.mcpcar.sit_down",
                       "蹲下",
                       PropertyList(), [this](const PropertyList &)
                       {
                           ESP_LOGI(TAG, "蹲下");
                           send_uart_command(0x30);
                           return true; });

    mcp_server.AddTool("self.mcpcar.stand_up",
                       "直立",
                       PropertyList(), [this](const PropertyList &)
                       {
                           ESP_LOGI(TAG, "直立");
                           send_uart_command(0x31);
                           return true; });

    mcp_server.AddTool("self.mcpcar.lay_down",
                       "趴下",
                       PropertyList(), [this](const PropertyList &)
                       {
                           ESP_LOGI(TAG, "趴下");
                           send_uart_command(0x32);
                           return true; });

    mcp_server.AddTool("self.mcpcar.forward",
                       "前进",
                       PropertyList(), [this](const PropertyList &)
                       {
                           ESP_LOGI(TAG, "前进");
                           send_uart_command(0x33);
                           return true; });

    mcp_server.AddTool("self.mcpcar.backward",
                       "后退",
                       PropertyList(), [this](const PropertyList &)
                       {
                           ESP_LOGI(TAG, "后退");
                           send_uart_command(0x34);
                           return true; });

    mcp_server.AddTool("self.mcpcar.turn_left",
                       "左转",
                       PropertyList(), [this](const PropertyList &)
                       {
                           ESP_LOGI(TAG, "左转");
                           send_uart_command(0x35);
                           return true; });

    mcp_server.AddTool("self.mcpcar.turn_right",
                       "右转",
                       PropertyList(), [this](const PropertyList &)
                       {
                           ESP_LOGI(TAG, "右转");
                           send_uart_command(0x36);
                           return true; });

    mcp_server.AddTool("self.mcpcar.swing",
                       "摇摆",
                       PropertyList(), [this](const PropertyList &)
                       {
                           ESP_LOGI(TAG, "摇摆");
                           send_uart_command(0x37);
                           return true; });

    mcp_server.AddTool("self.mcpcar.speed_up",
                       "移动加速",
                       PropertyList(), [this](const PropertyList &)
                       {
                           ESP_LOGI(TAG, "移动加速");
                           send_uart_command(0x38);
                           return true; });

    mcp_server.AddTool("self.mcpcar.swing_faster",
                       "摇摆加速",
                       PropertyList(), [this](const PropertyList &)
                       {
                           ESP_LOGI(TAG, "摇摆加速");
                           send_uart_command(0x39);
                           return true; });

    mcp_server.AddTool("self.mcpcar.wag_tail",
                       "摇尾巴",
                       PropertyList(), [this](const PropertyList &)
                       {
                           ESP_LOGI(TAG, "摇尾巴");
                           send_uart_command(0x40);
                           return true; });

    mcp_server.AddTool("self.mcpcar.jump_forward",
                       "向前跳",
                       PropertyList(), [this](const PropertyList &)
                       {
                           ESP_LOGI(TAG, "向前跳");
                           send_uart_command(0x41);
                           return true; });

    mcp_server.AddTool("self.mcpcar.jump_backward",
                       "向后跳",
                       PropertyList(), [this](const PropertyList &)
                       {
                           ESP_LOGI(TAG, "向后跳");
                           send_uart_command(0x42);
                           return true; });

    mcp_server.AddTool("self.mcpcar.greet",
                       "打招呼",
                       PropertyList(), [this](const PropertyList &)
                       {
                           ESP_LOGI(TAG, "打招呼");
                           send_uart_command(0x43);
                           return true; });

    mcp_server.AddTool("self.mcpcar.led_on",
                       "开灯",
                       PropertyList(), [this](const PropertyList &)
                       {
                           ESP_LOGI(TAG, "开灯");
                           send_uart_command(0x44);
                           return true; });

    mcp_server.AddTool("self.mcpcar.led_off",
                       "关灯",
                       PropertyList(), [this](const PropertyList &)
                       {
                           ESP_LOGI(TAG, "关灯");
                           send_uart_command(0x45);
                           return true; });

    mcp_server.AddTool("self.mcpcar.breathe_on",
                       "开启呼吸灯",
                       PropertyList(), [this](const PropertyList &)
                       {
                           ESP_LOGI(TAG, "呼吸灯开");
                           send_uart_command(0x46);
                           return true; });

    mcp_server.AddTool("self.mcpcar.breathe_off",
                       "关闭呼吸灯",
                       PropertyList(), [this](const PropertyList &)
                       {
                           ESP_LOGI(TAG, "呼吸灯关");
                           send_uart_command(0x47);
                           return true; });

    mcp_server.AddTool("self.mcpcar.stretch",
                       "伸懒腰",
                       PropertyList(), [this](const PropertyList &)
                       {
                           ESP_LOGI(TAG, "伸懒腰");
                           send_uart_command(0x48);
                           return true; });

    mcp_server.AddTool("self.mcpcar.stretch_leg",
                       "拉伸腿",
                       PropertyList(), [this](const PropertyList &)
                       {
                           ESP_LOGI(TAG, "拉伸腿");
                           send_uart_command(0x49);
                           return true; });
    mcp_server.AddTool("self.mcpcar.fan_on",
                       "打开风扇",
                       PropertyList(), [this](const PropertyList &)
                       {
                           ESP_LOGI(TAG, "打开风扇");
                           send_uart_command(0x4A);
                           return true; });

    mcp_server.AddTool("self.mcpcar.cooling_on",
                       "打开散热",
                       PropertyList(), [this](const PropertyList &)
                       {
                           ESP_LOGI(TAG, "打开散热");
                           send_uart_command(0x4B);
                           return true; });

}

/* ---------------- 发指令 ---------------- */
void McpCar::send_uart_command(uint8_t cmd)
{
    if (!uart_ok_)
    {
        ESP_LOGW(TAG, "UART not ready, drop 0x%02X", cmd);
        return;
    }
    uart_write_bytes(UART_NUM_1, &cmd, 1);
}
/* 文件尾部：只保留一份，且去掉 static */
McpCar *g_car = nullptr; // 定义，全局可见

void McpCar::begin() // 唯一实现
{
    if (uart_ok_)
        return;

    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UART1_TXD, UART1_RXD,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    uart_ok_ = true;
    ESP_LOGI(TAG, "UART1 ready");
}

McpCar &McpCar::getInstance()
{
    assert(g_car && "call begin() first");
    return *g_car;
}


// 以 UTF-8 文本方式发送一串字符（如用户说出的数字）
void McpCar::SendUtf8(const std::string &utf8)
{
    if (!uart_ok_)
    {
        ESP_LOGW(TAG, "UART not ready, drop text: %s", utf8.c_str());
        return;
    }
    if (!utf8.empty()) {
        uart_write_bytes(UART_NUM_1, utf8.data(), utf8.size());
    }
}
