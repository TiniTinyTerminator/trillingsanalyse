/**
 * @file ADS1258.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-09-12
 *
 * @copyright Copyright (c) 2023
 *
 */

// linux
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

// spi required headers
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

// gpio required headers
#include <cstring> // for std::string(strerror
#include <cerrno>  // for errno

#include <fstream>

#include "ADS1258.h"
#include "commands.h"

void Ads1258::init_spi(void)
{
    int err;

    spi_fd = open("/dev/spidev0", O_RDWR | O_NONBLOCK);
    if (spi_fd < 0)
        throw std::runtime_error("Could not open SPI interface");

    err = ioctl(spi_fd, SPI_IOC_RD_MODE32, &SPI_MODE);

    if (err)
        throw std::runtime_error("error occured with configuring spi interface: " + std::string(strerror(err)));

    err = ioctl(spi_fd, SPI_IOC_WR_MODE32, &SPI_MODE);

    if (err)
        throw std::runtime_error("error occured with configuring spi interface: " + std::string(strerror(err)));

    err = ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &SPI_NBITS);

    if (err)
        throw std::runtime_error("error occured with configuring spi interface: " + std::string(strerror(err)));

    err = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &SPI_NBITS);

    if (err)
        throw std::runtime_error("error occured with configuring spi interface: " + std::string(strerror(err)));

    err = ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &SPI_SPEED);

    if (err)
        throw std::runtime_error("error occured with configuring spi interface: " + std::string(strerror(err)));

    err = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &SPI_SPEED);

    if (err)
        throw std::runtime_error("error occured with configuring spi interface: " + std::string(strerror(err)));
}

void Ads1258::deinit_spi(void)
{
    int err = close(spi_fd);

    if (err)
        throw std::runtime_error("error occured with de initializing spi interface: " + std::string(strerror(err)));
}

std::vector<char> Ads1258::transieve_spi(std::vector<char> tx)
{
    std::lock_guard guard(spi_mutex);

    std::vector<char> rx(tx.size(), 0);

    lines_out->set_value(Pins::CS, ACTIVE);

    spi_ioc_transfer transfer = {
        .tx_buf = (unsigned long long)(tx.data()),
        .rx_buf = (unsigned long long)(rx.data()),
        .len = tx.size(),
        .speed_hz = (unsigned int)SPI_SPEED,
        .bits_per_word = 8};

    if (!spi_fd)
        throw std::runtime_error("SPI file not found");

    int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &transfer);

    if (ret < 1)
        throw std::runtime_error("Could not transmit message");

    lines_out->set_value(Pins::CS, INACTIVE);

    return rx;
}

void Ads1258::transmit_spi(std::vector<char> tx)
{
    std::lock_guard guard(spi_mutex);

    lines_out->set_value(Pins::CS, ACTIVE);

    spi_ioc_transfer transfer = {
        .tx_buf = (unsigned long long)(tx.data()),
        .rx_buf = (unsigned long long)(nullptr),
        .len = tx.size(),
        .speed_hz = (unsigned int)SPI_SPEED,
        .bits_per_word = 8};

    if (!spi_fd)
        throw std::runtime_error("spi file not found");

    int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &transfer);

    if (ret < 1)
        throw std::runtime_error("could not transmit message");

    lines_out->set_value(Pins::CS, INACTIVE);
}

void Ads1258::init_gpio(void)
{
    *gpio = gpiod::chip("/dev/gpiochip0");

    *lines_out = gpio->prepare_request()
                     .set_consumer("ads1258-gpio-out")
                     .add_line_settings(
                         {Pins::PWDN, Pins::START, Pins::RESET, Pins::CS, Pins::CLKSEL, Pins::CLKIO},
                         gpiod::line_settings()
                             .set_direction(gpiod::line::direction::OUTPUT)
                             .set_drive(gpiod::line::drive::PUSH_PULL)
                             .set_output_value(gpiod::line::value::INACTIVE))
                     .do_request();

    *line_irq = gpio->prepare_request()
                    .set_consumer("ads1258-drdy-irq")
                    .add_line_settings(
                        Pins::DRDY,
                        gpiod::line_settings()
                            .set_direction(gpiod::line::direction::INPUT)
                            .set_bias(gpiod::line::bias::PULL_UP)
                            .set_edge_detection(gpiod::line::edge::RISING) // TODO check if falling or rising edge
                        )
                    .do_request();

    auto poll_thread = [&]()
    {
        gpiod::edge_event_buffer event_buffer(5);

        while (!is_finished)
        {

            if (line_irq->wait_edge_events(std::chrono::nanoseconds(50)))
            {
                ssize_t n_events = line_irq->read_edge_events(event_buffer);

                for (const auto event : event_buffer)
                {

                    gpiod::line::offset o = event.line_offset();

                    gpiod::edge_event::event_type t = event.type();
                }
            }
            // TODO add receive data code
        }
    };

    fut = std::async(std::launch::async, poll_thread);
}

void Ads1258::deinit_gpio(void)
{

    is_finished = true;

    fut.wait();

    gpio->close();
}

void Ads1258::write_gpio(Pins pin, gpiod::line::value val)
{
    lines_out->set_value(pin, val);
}

void Ads1258::write_gpio(gpiod::line::offsets pins, gpiod::line::values vals)
{
    lines_out->set_values(pins, vals);
}

gpiod::line::value Ads1258::read_gpio(Pins pin)
{
    return lines_out->get_value(pin);
}

gpiod::line::values Ads1258::read_gpio(gpiod::line::offsets pins)
{
    return lines_out->get_values(pins);
}

void Ads1258::reset_registers(void)
{

    registers[CONFIG0] = CONFIG0_DEFAULT;
    registers[CONFIG1] = CONFIG1_DEFAULT;
    registers[MUXSCH] = MUXSCH_DEFAULT;
    registers[MUXDIF] = MUXDIF_DEFAULT;
    registers[MUXSG0] = MUXSG0_DEFAULT;
    registers[MUXSG1] = MUXSG1_DEFAULT;
    registers[GPIOC] = GPIOC_DEFAULT;
    registers[GPIOD] = GPIOD_DEFAULT;

    write_all_registers();
}

void Ads1258::write_all_registers()
{
    std::vector<char> tx = {OPCODE_WREG & OPCODE_C_MASK | OPCODE_MUL_MASK | 0x0 & OPCODE_A_MASK};

    for (const auto [key, reg] : registers)
    {
        tx.push_back(reg);
    }

    transmit_spi(tx);
}

void Ads1258::read_all_registers()
{
    std::vector<char> tx(registers.size(), 0);

    tx[0] = OPCODE_RREG & OPCODE_C_MASK | OPCODE_MUL_MASK | 0x0 & OPCODE_A_MASK;

    std::vector<char> rx = transieve_spi(tx);

    for (size_t i = 0; i < rx.size(); i++)
    {
        registers[(Registers)i] = rx[i];
    }
}

void Ads1258::write_register(Registers addr_start, char data)
{
    if (addr_start > Registers::ID)
        throw std::invalid_argument("invalid register address");

    registers[addr_start] = data;

    std::vector<char> tx = {OPCODE_WREG & OPCODE_C_MASK | addr_start & OPCODE_A_MASK};

    tx.push_back(data);

    transmit_spi(tx);
}

void Ads1258::write_register(Registers addr_start, std::vector<char> data)
{
    if (addr_start + data.size() > Registers::ID)
        throw std::invalid_argument("invalid register address");

    for (size_t addr = addr_start; addr < (addr + data.size()); addr++)
        registers[(Registers)addr] = data[addr - addr_start];

    std::vector<char> tx = {OPCODE_WREG & OPCODE_C_MASK | addr_start & OPCODE_A_MASK};

    tx.reserve(data.size() + 1);

    std::copy(data.begin(), data.end(), tx.begin() + 1);

    transmit_spi(tx);
}

bool Ads1258::check_registers(void)
{
    std::vector<char> tx(registers.size(), 0);

    tx[0] = OPCODE_RREG & OPCODE_C_MASK | OPCODE_MUL_MASK | 0x0 & OPCODE_A_MASK;

    std::vector<char> rx = transieve_spi(tx);

    for (size_t i = 0; i < rx.size(); i++)
    {
        if(registers[(Registers)i] != rx[i])
            return false;
    }   

    return true;
}

void Ads1258::setup_device(void)
{

    registers[CONFIG0] = 0b01110110;
    registers[CONFIG1] = 0b00000010;

    // fixed-scan channel select
    registers[MUXSCH] = 0x00;

    // autoscan differential channel select
    registers[MUXDIF] = 0x00;

    // autoscan channel select
    registers[MUXSG0] = 0xFF;
    registers[MUXSG1] = 0xFF;

    registers[GPIOC] = 0xFF;
    registers[GPIOD] = 0x00;

    write_gpio(Pins::CLKIO, INACTIVE);
    write_gpio(Pins::PWDN, ACTIVE);
    write_gpio(Pins::RESET, INACTIVE);
    write_gpio(Pins::START, INACTIVE);
    write_gpio(Pins::CLKSEL, INACTIVE);

    write_all_registers();


}

void Ads1258::conversion_stop()
{
    write_gpio(Pins::START, INACTIVE);
}

void Ads1258::conversion_start()
{
    write_gpio(Pins::START, ACTIVE);
}

Ads1258::~Ads1258()
{
    deinit_spi();
    deinit_gpio();
}

Ads1258::Ads1258() : SPI_SPEED(16e6), SPI_NBITS(8), SPI_MODE(SPI_MODE_0), SPI_BUFF_SIZE(128)
{
    init_gpio();
    init_spi();
}