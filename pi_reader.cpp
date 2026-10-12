// pi_reader.cpp — decode Pi<->STM32 frames directly from Windows, no Pi needed yet.
// Build (MSVC):   cl pi_reader.cpp
// Build (MinGW):  g++ pi_reader.cpp -o pi_reader.exe
// Run:            pi_reader.exe COM5

#include <windows.h>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

static constexpr uint8_t SYNC    = 0xA5;
static constexpr uint8_t MSG_IMU = 0x01;

static uint16_t crc16CcittFalse(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s COM5\n", argv[0]);
        return 1;
    }

    std::string portPath = std::string("\\\\.\\") + argv[1];
    HANDLE hSerial = CreateFileA(portPath.c_str(), GENERIC_READ | GENERIC_WRITE,
                                  0, NULL, OPEN_EXISTING, 0, NULL);
    if (hSerial == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Failed to open %s (error %lu)\n", argv[1], GetLastError());
        return 1;
    }

    DCB dcb = {};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(hSerial, &dcb)) { fprintf(stderr, "GetCommState failed\n"); return 1; }
    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    if (!SetCommState(hSerial, &dcb)) { fprintf(stderr, "SetCommState failed\n"); return 1; }

    COMMTIMEOUTS timeouts = {};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    SetCommTimeouts(hSerial, &timeouts);
    PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR); 

    std::vector<uint8_t> buf;
    uint8_t chunk[256];
    int lastSeq = -1;

    for (;;) {
        DWORD bytesRead = 0;
        if (!ReadFile(hSerial, chunk, sizeof(chunk), &bytesRead, NULL)) {
            fprintf(stderr, "ReadFile failed\n");
            break;
        }
        if (bytesRead > 0) buf.insert(buf.end(), chunk, chunk + bytesRead);

        for (;;) {
            size_t idx = 0;
            bool found = false;
            for (; idx < buf.size(); idx++) { if (buf[idx] == SYNC) { found = true; break; } }
            if (!found) { buf.clear(); break; }
            if (idx > 0) buf.erase(buf.begin(), buf.begin() + idx);

            if (buf.size() < 3) break;  // need sync+type+length at least

            uint8_t msgType = buf[1];
            uint8_t length  = buf[2];
            size_t frameLen = 3 + length + 2;

            if (buf.size() < frameLen) break;  // wait for more bytes

            const uint8_t* payload = &buf[3];
            uint16_t crcRx = buf[3 + length] | (static_cast<uint16_t>(buf[3 + length + 1]) << 8);
            uint16_t crcCalc = crc16CcittFalse(&buf[1], 2 + length);  // type+length+payload

            if (crcRx != crcCalc) {
                buf.erase(buf.begin());  // bad frame: drop just the sync byte, resync
                continue;
            }

            if (msgType == MSG_IMU) {
                uint8_t seq = payload[0];
                uint32_t ts; float yaw, gx, gy, gz, ax, ay, az;
                memcpy(&ts,  payload + 1,  4);
                memcpy(&yaw, payload + 5,  4);
                memcpy(&gx,  payload + 9,  4);
                memcpy(&gy,  payload + 13, 4);
                memcpy(&gz,  payload + 17, 4);
                memcpy(&ax,  payload + 21, 4);
                memcpy(&ay,  payload + 25, 4);
                memcpy(&az,  payload + 29, 4);

                char gap[32] = "";
                if (lastSeq >= 0) {
                    int diff = (seq - lastSeq) & 0xFF;
                    if (diff != 1) snprintf(gap, sizeof(gap), "  [lost %d]", diff - 1);
                }
                lastSeq = seq;

                printf("seq=%3u t=%8u ms yaw=%+.3f gyro=(%+.3f,%+.3f,%+.3f) accel=(%+.2f,%+.2f,%+.2f)%s\n",
                       seq, ts, yaw, gx, gy, gz, ax, ay, az, gap);
            }

            buf.erase(buf.begin(), buf.begin() + frameLen);  // consume the whole frame
        }
    }

    CloseHandle(hSerial);
    return 0;
}