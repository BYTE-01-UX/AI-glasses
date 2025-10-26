#ifndef MCPCAR_H
#define MCPCAR_H

#include "mcp_server.h"
#include "driver/uart.h"
#include <driver/gpio.h>
#include <esp_log.h>
#pragma once
#include <cstdint>
#include <string>

// ML307 串口引脚定义（按需求指定到 44/43）
#define ML307_RX_PIN GPIO_NUM_44
#define ML307_TX_PIN GPIO_NUM_43

// class McpCar
// {
// private:
//     void uart1_init(void);
//     void send_uart_command(uint8_t command);

// public:
//     McpCar();
// };
class McpCar
{
public:
    McpCar();                     // 构造函数（只注册工具）
    void begin();                 // 硬件初始化
    static McpCar &getInstance(); // 单例访问

    // 以 UTF-8 文本方式发送（例如把用户说出的数字串直接发出去）
    void SendUtf8(const std::string &utf8);

private:
    void send_uart_command(uint8_t cmd);
    bool uart_ok_{false}; // ① 加上成员变量
};

/* 如果别处想直接拿指针，也可以把 g_car 暴露在这里 */
extern McpCar *g_car;

#endif // MCPCAR_H
