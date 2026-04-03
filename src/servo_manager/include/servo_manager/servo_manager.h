#pragma once

#include "common/singleton.h"
#include "servo_driver/SCServo.h"
#include <atomic>
#include <chrono>
#include <iostream>
#include <math.h>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <vector>

class ServoManager : public Singleton<ServoManager> {

    friend class Singleton<ServoManager>;

  protected:
    ServoManager() { init(); };
    virtual ~ServoManager() { sm_st_.end(); };

  public:
    ServoManager(const ServoManager &) = delete;
    ServoManager &operator=(const ServoManager &) = delete;

    bool init(const std::string &serial_port = "/dev/ttyACM0", int baudRate = 1000000);
    void close();

    void stop(std::vector<uint8_t> &ids, uint8_t run_mode);
    bool move(std::vector<uint8_t> &ids, std::vector<double> &position, std::vector<double> &speed, std::vector<double> &acceleration);
    bool get_state(std::vector<uint8_t> &ids, std::vector<double> &position, std::vector<double> &speed);
    bool enable_torque(std::vector<uint8_t> &ids, bool enable);

    bool calibration_ofs(std::vector<uint8_t> &ids);

    bool ping(uint8_t id);
    bool set_id(uint8_t old_id, uint8_t new_id);
    bool calibration_ofs(uint8_t id);

    bool write_byte(uint8_t id, uint8_t addr, uint8_t value, bool is_eprom = true);
    int read_byte(uint8_t id, uint8_t addr);

    bool write_word(uint8_t id, uint8_t addr, int value, bool is_eprom = true, uint8_t negBit = 0);
    int read_word(uint8_t id, uint8_t addr, uint8_t negBit = 0);

    int write_addr(uint8_t id, uint8_t addr, uint8_t *data, uint8_t len, bool is_eprom = true);
    int read_addr(uint8_t id, uint8_t addr, uint8_t *data, uint8_t len);

    int merge_two_byte(uint8_t DataL, uint8_t DataH, uint8_t negBit = 0);

  private:
    SMS_STS sm_st_;
    std::atomic<bool> is_initialized_{false};
    std::shared_mutex shared_mutex_;

    int _handle_word(int value, uint8_t negBit);
};