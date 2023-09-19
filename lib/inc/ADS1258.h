/**
 * @file ADS1258.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-09-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <atomic>
#include <map>
#include <mutex>
#include <thread>
#include <future>
#include <queue>
#include <iostream>

#include <gpiod.hpp>

#include "commands.h"

class Ads1258
{

private:
    enum Registers
    {
        CONFIG0,
        CONFIG1,
        MUXSCH,
        MUXDIF,
        MUXSG0,
        MUXSG1,
        SYSRED,
        GPIOC,
        GPIOD,
        ID
    };

    enum Pins
    {
        CS = 8,
        DRDY = 27,
        CLKSEL = 17,
        START = 15,
        CLKIO = 7,
        RESET = 12,
        PWDN = 16
    };

    using enum gpiod::line::value;

    std::unique_ptr<gpiod::chip> gpio;
    std::unique_ptr<gpiod::line_request> lines_out, line_irq;

    std::map<Registers, char> registers;

    int SPI_SPEED;
    int SPI_NBITS;
    int SPI_MODE;
    int SPI_BUFF_SIZE;

    int spi_fd;

    std::mutex spi_mutex;
    std::atomic<bool> is_finished;
    std::queue<std::pair<uint8_t, uint32_t>> adc_data;
    std::future<void> fut;

    void init_gpio(void);
    void deinit_gpio(void);
 
    inline gpiod::line::value read_gpio(Pins pin);
    inline gpiod::line::values read_gpio(gpiod::line::offsets pins);

    inline void write_gpio(Pins pin, gpiod::line::value val);
    inline void write_gpio(gpiod::line::offsets pins, gpiod::line::values val);

    void init_spi(void);
    void deinit_spi(void);

    void reset_registers(void);
    void write_all_registers(void);
    void read_all_registers(void);
    bool check_registers(void);
    
public:
    void setup_device(void);

    void conversion_stop();
    void conversion_start();

    void write_register(Registers addr_start, char data);
    void write_register(Registers addr_start, std::vector<char> data);

    std::vector<char> transieve_spi(std::vector<char> tx);

    void transmit_spi(std::vector<char> tx);

    ~Ads1258();
    Ads1258();
};