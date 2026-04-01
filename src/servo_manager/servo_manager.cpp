#include "servo_manager.h"

void ServoManager::init(const std::string &serial_port, int baudRate) {
    std::unique_lock lock(shared_mutex_);
    is_initialized_.store(false);
    sm_st_.end();
    if (!sm_st_.begin(baudRate, serial_port.c_str())) {
        std::cerr << "Failed to initialize SMS_STS motor!" << std::endl;
    } else {
        std::cout << "SMS_STS motor initialized successfully!" << std::endl;
        is_initialized_.store(true);
    }
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