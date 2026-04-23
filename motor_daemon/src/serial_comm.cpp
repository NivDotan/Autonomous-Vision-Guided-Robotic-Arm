#include "serial_comm.hpp"
#include <cstring>
#include <stdexcept>
#include <cstdio>

#ifdef _WIN32
#  include <windows.h>
#else
#  include <fcntl.h>
#  include <termios.h>
#  include <unistd.h>
#  include <sys/ioctl.h>
#endif

// ── Feetech STS3215 protocol constants ───────────────────────────────────────
static constexpr uint8_t HEADER          = 0xFF;
static constexpr uint8_t INST_READ       = 0x02;
static constexpr uint8_t INST_WRITE      = 0x03;
static constexpr uint8_t INST_SYNC_WRITE = 0x83;
static constexpr uint8_t INST_SYNC_READ  = 0x82;

// STS3215 register addresses.
static constexpr uint8_t REG_GOAL_POS         = 42;  // 2 bytes, little-endian
static constexpr uint8_t REG_PRESENT_POS      = 56;  // 2 bytes
static constexpr uint8_t REG_PRESENT_LOAD     = 60;  // 2 bytes

// ── Platform-specific serial I/O ─────────────────────────────────────────────

SerialComm::SerialComm() : sim_mode_(false) {
#ifdef _WIN32
    handle_ = INVALID_HANDLE_VALUE;
#else
    fd_ = -1;
#endif
}

SerialComm::~SerialComm() { close(); }

bool SerialComm::open_sim() {
    sim_mode_ = true;
    // Initialise simulated positions to mid-range (2048).
    sim_pos_.fill(2048);
    sim_load_.fill(0);
    return true;
}

bool SerialComm::open(const std::string& port, int baud) {
#ifdef _WIN32
    std::string dev = "\\\\.\\" + port;
    handle_ = CreateFileA(dev.c_str(), GENERIC_READ | GENERIC_WRITE,
                          0, nullptr, OPEN_EXISTING, 0, nullptr);
    if (handle_ == INVALID_HANDLE_VALUE) return false;

    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(handle_, &dcb);
    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity   = NOPARITY;
    SetCommState(handle_, &dcb);

    COMMTIMEOUTS to{};
    to.ReadIntervalTimeout         = 10;
    to.ReadTotalTimeoutConstant    = 50;
    to.WriteTotalTimeoutConstant   = 50;
    SetCommTimeouts(handle_, &to);
    return true;
#else
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    struct termios tty{};
    tcgetattr(fd_, &tty);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;  // 100ms timeout
    tcsetattr(fd_, TCSANOW, &tty);
    return true;
#endif
}

void SerialComm::close() {
#ifdef _WIN32
    if (handle_ != INVALID_HANDLE_VALUE) {
        CloseHandle(handle_);
        handle_ = INVALID_HANDLE_VALUE;
    }
#else
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
#endif
}

// Raw byte write/read helpers.
static void serial_write(
#ifdef _WIN32
    HANDLE h,
#else
    int fd,
#endif
    const uint8_t* buf, int len) {
#ifdef _WIN32
    DWORD written;
    WriteFile(h, buf, len, &written, nullptr);
#else
    ::write(fd, buf, len);
#endif
}

static int serial_read(
#ifdef _WIN32
    HANDLE h,
#else
    int fd,
#endif
    uint8_t* buf, int len, int timeout_ms = 50) {
    int total = 0;
#ifdef _WIN32
    DWORD rd;
    while (total < len) {
        if (!ReadFile(h, buf + total, len - total, &rd, nullptr)) break;
        total += rd;
        if (rd == 0) break;
    }
#else
    (void)timeout_ms;
    int rd;
    while (total < len) {
        rd = ::read(fd, buf + total, len - total);
        if (rd <= 0) break;
        total += rd;
    }
#endif
    return total;
}

// ── Packet construction ───────────────────────────────────────────────────────

static uint8_t checksum(const uint8_t* data, int len) {
    uint16_t sum = 0;
    for (int i = 0; i < len; ++i) sum += data[i];
    return static_cast<uint8_t>(~sum & 0xFF);
}

// ── Torque enable/disable (individual write per motor) ───────────────────────
// STS3215 register 40 = Torque_Enable, 1 byte.
static constexpr uint8_t REG_TORQUE_ENABLE = 40;

void SerialComm::torque_enable_all(const std::array<int, 6>& ids, bool enable) {
    if (sim_mode_) return;
    uint8_t val = enable ? 1 : 0;
    for (int id : ids) {
        uint8_t pkt[8];
        pkt[0] = HEADER;
        pkt[1] = HEADER;
        pkt[2] = static_cast<uint8_t>(id);
        pkt[3] = 4;               // length = 3 + data_len
        pkt[4] = INST_WRITE;
        pkt[5] = REG_TORQUE_ENABLE;
        pkt[6] = val;
        pkt[7] = checksum(pkt + 2, 5);
#ifdef _WIN32
        serial_write(handle_, pkt, 8);
        uint8_t resp[6]; serial_read(handle_, resp, 6);  // consume status packet
#else
        serial_write(fd_, pkt, 8);
        uint8_t resp[6]; serial_read(fd_, resp, 6);
#endif
    }
}

// ── Individual read: Present_Position for one motor ──────────────────────────
// More reliable than SYNC_READ across firmware versions.
static bool read_position_single(
#ifdef _WIN32
    HANDLE h,
#else
    int fd,
#endif
    int motor_id, int16_t& pos_out)
{
    // Request: FF FF ID 04 02 REG_L DATA_LEN CS
    uint8_t pkt[8];
    pkt[0] = HEADER;
    pkt[1] = HEADER;
    pkt[2] = static_cast<uint8_t>(motor_id);
    pkt[3] = 4;           // LEN = params(2) + 2
    pkt[4] = INST_READ;
    pkt[5] = REG_PRESENT_POS;
    pkt[6] = 2;           // read 2 bytes
    pkt[7] = checksum(pkt + 2, 5);
#ifdef _WIN32
    serial_write(h, pkt, 8);
    uint8_t resp[8];
    int got = serial_read(h, resp, 8);
#else
    serial_write(fd, pkt, 8);
    uint8_t resp[8];
    int got = serial_read(fd, resp, 8);
#endif
    if (got < 8 || resp[0] != HEADER || resp[1] != HEADER) return false;
    pos_out = static_cast<int16_t>(resp[5] | (resp[6] << 8));
    return true;
}

bool SerialComm::sync_read_positions(const std::array<int, 6>& ids, Ticks6& out) {
    if (sim_mode_) {
        for (int i = 0; i < 6; ++i) out[i] = sim_pos_[i];
        return true;
    }
    bool ok = true;
    for (int i = 0; i < 6; ++i) {
        int16_t pos = out[i];  // keep last value on failure
#ifdef _WIN32
        if (!read_position_single(handle_, ids[i], pos)) ok = false;
#else
        if (!read_position_single(fd_, ids[i], pos)) ok = false;
#endif
        out[i] = pos;
    }
    return ok;
}

// ── Sync write: write Goal_Position to all motors ───────────────────────────

bool SerialComm::sync_write_positions(const std::array<int, 6>& ids, const Ticks6& ticks) {
    if (sim_mode_) {
        sim_pos_ = ticks;
        return true;
    }

    const int n = 6;
    // Packet layout: FF FF FE LEN INST ADDR DATA_LEN [ID POS_L POS_H]×n CS
    // LEN  = INST(1) + ADDR(1) + DATA_LEN(1) + n*(ID+2) + CS(1) = 4 + n*3
    // Total bytes = 2(header) + 1(FE) + 1(LEN) + 1(INST) + 1(ADDR) + 1(DATA_LEN)
    //             + n*3(motor data) + 1(CS) = 7 + n*3 = 25 for n=6 → but +1 header = 26
    // Indices: [0,1]=FF FF  [2]=FE  [3]=LEN  [4]=INST  [5]=ADDR  [6]=DATA_LEN
    //          [7..7+n*3-1]=motor data   [7+n*3]=CS
    const int total = 7 + n * 3 + 1;  // = 26 bytes
    uint8_t pkt[32] = {};
    pkt[0] = HEADER;
    pkt[1] = HEADER;
    pkt[2] = 0xFE;                              // broadcast ID
    pkt[3] = static_cast<uint8_t>(4 + n * 3);  // LEN = 22
    pkt[4] = INST_SYNC_WRITE;
    pkt[5] = REG_GOAL_POS;
    pkt[6] = 2;  // 2 bytes per motor

    for (int i = 0; i < n; ++i) {
        pkt[7 + i * 3]     = static_cast<uint8_t>(ids[i]);
        pkt[7 + i * 3 + 1] = static_cast<uint8_t>(ticks[i] & 0xFF);
        pkt[7 + i * 3 + 2] = static_cast<uint8_t>((ticks[i] >> 8) & 0xFF);
    }
    // Checksum covers pkt[2..total-2] (ID, LEN, INST, ADDR, DATA_LEN, motor data)
    pkt[total - 1] = checksum(pkt + 2, total - 3);

#ifdef _WIN32
    serial_write(handle_, pkt, total);
#else
    serial_write(fd_, pkt, total);
#endif
    return true;
}

// ── Read gripper load ────────────────────────────────────────────────────────

int16_t SerialComm::read_load(int motor_id) {
    if (sim_mode_) return sim_load_[5];

    // Single-motor READ for Present_Load (addr=60, len=2).
    uint8_t pkt[8];
    pkt[0] = HEADER;
    pkt[1] = HEADER;
    pkt[2] = static_cast<uint8_t>(motor_id);
    pkt[3] = 4;
    pkt[4] = INST_READ;
    pkt[5] = REG_PRESENT_LOAD;
    pkt[6] = 2;
    pkt[7] = checksum(pkt + 2, 5);

#ifdef _WIN32
    serial_write(handle_, pkt, 8);
    uint8_t resp[8];
    int got = serial_read(handle_, resp, 8);
#else
    serial_write(fd_, pkt, 8);
    uint8_t resp[8];
    int got = serial_read(fd_, resp, 8);
#endif

    if (got < 8 || resp[0] != HEADER || resp[1] != HEADER) return 0;
    int16_t raw = static_cast<int16_t>(resp[5] | (resp[6] << 8));
    // Convert: if raw > 1024, subtract 1024 to get signed load.
    return (raw > 1024) ? raw - 1024 : raw;
}
