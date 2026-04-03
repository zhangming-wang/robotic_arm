#include "servo_manager/servo_manager.h"

bool ServoManager::init(const std::string &serial_port, int baudRate) {
    close();
    std::unique_lock lock(shared_mutex_);
    if (!sm_st_.begin(baudRate, serial_port.c_str())) {
        std::cerr << "Failed to initialize SMS_STS motor!" << std::endl;
        return false;
    } else {
        std::cout << "SMS_STS motor initialized successfully!" << std::endl;
        is_initialized_.store(true);
        return true;
    }
}

void ServoManager::close() {
    std::unique_lock lock(shared_mutex_);
    is_initialized_.store(false);
    sm_st_.end();
}

void ServoManager::stop(std::vector<uint8_t> &ids, uint8_t run_mode) {
    if (!is_initialized_.load()) {
        std::cerr << "not initialized, stop failed!" << std::endl;
        return;
    }

    if (run_mode == 0) {
        std::vector<uint8_t> data(ids.size(), 0);
        std::unique_lock lock(shared_mutex_);
        sm_st_.snycWrite(ids.data(), ids.size(), SMSBL_TORQUE_ENABLE, data.data(), data.size());
    } else if (run_mode == 1) {
        std::vector<int16_t> servo_pos(ids.size(), 0), servo_speed(ids.size(), 0);
        std::vector<uint8_t> servo_acc(ids.size(), 0);
        std::unique_lock lock(shared_mutex_);
        sm_st_.SyncWritePosEx(ids.data(), ids.size(), servo_pos.data(), servo_speed.data(), servo_acc.data());
    } else {
        std::cerr << "unsupported run_mode: " << (int)run_mode << std::endl;
    }
}

bool ServoManager::move(std::vector<uint8_t> &ids, std::vector<double> &position, std::vector<double> &speed, std::vector<double> &acceleration) {
    if (!is_initialized_.load()) {
        std::cerr << "not initialized, move failed!" << std::endl;
        return false;
    }

    if (position.size() != ids.size() || speed.size() != ids.size() || acceleration.size() != ids.size()) {
        std::cerr << "size of position, speed, acceleration should be the same as size of ids!" << std::endl;
        return false;
    }

    std::vector<int16_t> servo_pos(position.size(), 0), servo_speed(speed.size(), 0);
    std::vector<uint8_t> servo_acc(acceleration.size(), 0);

    for (size_t i = 0; i < position.size(); ++i) {
        servo_pos[i] = static_cast<int16_t>(position[i] / M_PI * 2048 + 2048);
        servo_speed[i] = static_cast<int16_t>(speed[i] / M_PI * 2048);
        servo_acc[i] = static_cast<uint8_t>(acceleration[i] / M_PI * 2048);
    }

    std::unique_lock lock(shared_mutex_);

    sm_st_.SyncWritePosEx(ids.data(), ids.size(), servo_pos.data(), servo_speed.data(), servo_acc.data());

    return true;
}

bool ServoManager::get_state(std::vector<uint8_t> &ids, std::vector<double> &position, std::vector<double> &speed) {
    if (!is_initialized_.load()) {
        std::cerr << "not initialized, get_state failed!" << std::endl;
        return false;
    }

    if (position.size() != ids.size() || speed.size() != ids.size()) {
        std::cerr << "size of position, speed should be the same as size of ids!" << std::endl;
        return false;
    }

    std::vector<int16_t> servo_pos(position.size(), 0), servo_speed(speed.size(), 0);

    {
        std::unique_lock lock(shared_mutex_);
        bool success = sm_st_.SyncReadPosEx(ids.data(), ids.size(), servo_pos.data(), servo_speed.data());
        if (!success) {
            std::cerr << "Failed to read servo state!" << std::endl;
            return false;
        }
    }

    for (size_t i = 0; i < position.size(); ++i) {
        position[i] = static_cast<double>(servo_pos[i] - 2048) / 2048 * M_PI;
        speed[i] = static_cast<double>(servo_speed[i]) / 2048 * M_PI;
    }

    return true;
}

bool ServoManager::enable_torque(std::vector<uint8_t> &ids, bool enable) {
    if (!is_initialized_.load()) {
        std::cerr << "not initialized, enable_torque failed!" << std::endl;
        return false;
    }

    std::vector<uint8_t> data(ids.size(), enable ? 1 : 0);
    std::unique_lock lock(shared_mutex_);
    sm_st_.snycWrite(ids.data(), ids.size(), SMSBL_TORQUE_ENABLE, data.data(), data.size());
    return true;
}

bool ServoManager::calibration_ofs(std::vector<uint8_t> &ids) {
    if (!is_initialized_.load()) {
        std::cerr << "not initialized, calibration_ofs failed!" << std::endl;
        return false;
    }

    std::vector<uint8_t> data(ids.size(), 128);
    std::unique_lock lock(shared_mutex_);
    sm_st_.snycWrite(ids.data(), ids.size(), SMSBL_TORQUE_ENABLE, data.data(), data.size());
    return true;
}

bool ServoManager::ping(uint8_t id) {
    if (!is_initialized_.load()) {
        std::cerr << "not initialized, ping failed!" << std::endl;
        return false;
    }

    std::unique_lock lock(shared_mutex_);

    auto res = sm_st_.Ping(id);
    if (res != -1) {
        std::cout << "Ping id = " << res << " success!" << std::endl;
        return true;
    } else {
        std::cerr << "Ping id = " << res << " failed!" << std::endl;
        return false;
    }
}

bool ServoManager::set_id(uint8_t old_id, uint8_t new_id) {
    if (!is_initialized_.load()) {
        std::cerr << "not initialized, set_id failed!" << std::endl;
        return false;
    }

    std::unique_lock lock(shared_mutex_);

    if (!sm_st_.unLockEprom(old_id)) {
        std::cerr << "set_id: Failed to unLockEp" << std::endl;
        return false;
    }

    bool res = sm_st_.writeByte(old_id, SMSBL_ID, new_id);

    if (!sm_st_.LockEprom(new_id)) {
        std::cerr << "set_id: Failed to LockEprom" << std::endl;
        return false;
    }

    if (!res) {
        std::cerr << "set_id: Failed to writeByte" << std::endl;
    }

    return res;
}

bool ServoManager::calibration_ofs(uint8_t id) {
    if (!is_initialized_.load()) {
        std::cerr << "not initialized, set id failed!" << std::endl;
        return false;
    }

    std::unique_lock lock(shared_mutex_);

    if (!sm_st_.CalibrationOfs(id)) {
        std::cerr << "id = " << id << ": calibration_ofs fail" << std::endl;
        return false;
    }
    return true;
}

int ServoManager::read_addr(uint8_t id, uint8_t addr, uint8_t *data, uint8_t len) {
    if (!is_initialized_.load()) {
        std::cerr << "not initialized, read_addr failed!" << std::endl;
        return -1;
    }

    std::unique_lock lock(shared_mutex_);
    return sm_st_.Read(id, addr, data, len);
}

int ServoManager::write_addr(uint8_t id, uint8_t addr, uint8_t *data, uint8_t len, bool is_eprom) {
    if (!is_initialized_.load()) {
        std::cerr << "not initialized, write_addr failed!" << std::endl;
        return -1;
    }

    std::unique_lock lock(shared_mutex_);

    if (is_eprom) {
        if (!sm_st_.unLockEprom(id)) {
            std::cerr << "_write_addr: Failed to unLockEp" << std::endl;
            return false;
        }
    }

    bool res = sm_st_.regWrite(id, addr, data, len);

    if (is_eprom) {
        if (!sm_st_.LockEprom(id)) {
            std::cerr << "_write_addr: Failed to LockEprom" << std::endl;
            return false;
        }
    }

    if (!res) {
        std::cerr << "_write_addr: Failed!" << std::endl;
    }

    return res;
}

bool ServoManager::write_byte(uint8_t id, uint8_t addr, uint8_t value, bool is_eprom) {
    if (!is_initialized_.load()) {
        std::cerr << "not initialized, _write_byte failed!" << std::endl;
        return false;
    }

    std::unique_lock lock(shared_mutex_);

    if (is_eprom) {
        if (!sm_st_.unLockEprom(id)) {
            std::cerr << "_write_byte: Failed to unLockEp" << std::endl;
            return false;
        }
    }

    bool res = sm_st_.writeByte(id, addr, value);

    if (is_eprom) {
        if (!sm_st_.LockEprom(id)) {
            std::cerr << "_write_byte: Failed to LockEprom" << std::endl;
            return false;
        }
    }

    if (!res) {
        std::cerr << "_write_byte: Failed!" << std::endl;
    }

    return res;
}

int ServoManager::read_byte(uint8_t id, uint8_t addr) {
    if (!is_initialized_.load()) {
        std::cerr << "not initialized, _read_byte failed!" << std::endl;
        return -1;
    }

    std::unique_lock lock(shared_mutex_);
    return sm_st_.readByte(id, addr);
}

bool ServoManager::write_word(uint8_t id, uint8_t addr, int value, bool is_eprom, uint8_t negBit) {
    if (!is_initialized_.load()) {
        std::cerr << "not initialized, _write_word failed!" << std::endl;
        return false;
    }

    if (value < 0 && negBit != 0) {
        value = abs(value) | (1 << negBit);
    }

    std::unique_lock lock(shared_mutex_);
    if (is_eprom) {
        if (!sm_st_.unLockEprom(id)) {
            std::cerr << "_write_word: Failed to unLockEp" << std::endl;
            return false;
        }
    }

    bool res = sm_st_.writeWord(id, addr, value);

    if (is_eprom) {
        if (!sm_st_.LockEprom(id)) {
            std::cerr << "_write_word: Failed to LockEprom" << std::endl;
            return false;
        }
    }

    if (!res) {
        std::cerr << "_write_word: Failed!" << std::endl;
    }

    return res;
}

int ServoManager::read_word(uint8_t id, uint8_t addr, uint8_t negBit) {
    if (!is_initialized_.load()) {
        std::cerr << "not initialized, _read_word failed!" << std::endl;
        return -1;
    }

    std::unique_lock lock(shared_mutex_);
    return _handle_word(sm_st_.readWord(id, addr), negBit);
}

int ServoManager::merge_two_byte(uint8_t DataL, uint8_t DataH, uint8_t negBit) {
    return _handle_word(sm_st_.SCS2Host(DataL, DataH), negBit);
}

int ServoManager::_handle_word(int value, uint8_t negBit) {
    if (negBit != 0 && value != -1 && (value & (1 << negBit)) != 0) {
        value = -(value & ~(1 << negBit));
    }
    return value;
}