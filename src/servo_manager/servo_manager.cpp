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

bool ServoManager::write_world(uint8_t id, uint8_t addr, uint16_t value, bool is_eprom) {
    if (!is_initialized_.load()) {
        std::cerr << "not initialized, _write_world failed!" << std::endl;
        return false;
    }

    std::unique_lock lock(shared_mutex_);
    if (is_eprom) {
        if (!sm_st_.unLockEprom(id)) {
            std::cerr << "_write_world: Failed to unLockEp" << std::endl;
            return false;
        }
    }

    bool res = sm_st_.writeWord(id, addr, value);

    if (is_eprom) {
        if (!sm_st_.LockEprom(id)) {
            std::cerr << "_write_world: Failed to LockEprom" << std::endl;
            return false;
        }
    }

    if (!res) {
        std::cerr << "_write_world: Failed!" << std::endl;
    }

    return res;
}

int ServoManager::read_world(uint8_t id, uint8_t addr) {
    if (!is_initialized_.load()) {
        std::cerr << "not initialized, _read_world failed!" << std::endl;
        return -1;
    }

    std::unique_lock lock(shared_mutex_);
    return sm_st_.readWord(id, addr);
}

u16 ServoManager::merge_two_byte(u8 DataL, u8 DataH) {
    return sm_st_.SCS2Host(DataL, DataH);
}

void ServoManager::test() {
    int Pos;
    int Speed;
    int Load;
    int Voltage;
    int Temper;
    int Move;
    int Current;
    // 一条指令读舵机所有反馈数据至缓冲区
    if (sm_st_.FeedBack(1) != -1) {
        Pos = sm_st_.ReadPos(-1); //-1表示缓冲区数据，以下相同
        Speed = sm_st_.ReadSpeed(-1);
        Load = sm_st_.ReadLoad(-1);
        Voltage = sm_st_.ReadVoltage(-1);
        Temper = sm_st_.ReadTemper(-1);
        Move = sm_st_.ReadMove(-1);
        Current = sm_st_.ReadCurrent(-1);
        std::cout << "pos = " << Pos << " ";
        std::cout << "Speed = " << Speed << " ";
        std::cout << "Load = " << Load << " ";
        std::cout << "Voltage = " << Voltage << " ";
        std::cout << "Temper = " << Temper << " ";
        std::cout << "Move = " << Move << " ";
        std::cout << "Current = " << Current << std::endl;
        usleep(10 * 1000);
    } else {
        std::cout << "read err" << std::endl;
        sleep(1);
    }
}
