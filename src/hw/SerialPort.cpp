#include "hw/SerialPort.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace fs = std::filesystem;

namespace pegasus {

#ifndef _WIN32


namespace {

std::string readFirstLine(const fs::path& p) {
    std::ifstream f(p);
    std::string s;
    std::getline(f, s);
    return s;
}

speed_t baudConstant(int baud) {
    switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 500000: return B500000;
    case 1000000: return B1000000;
    default: return 0;
    }
}

} // namespace

std::vector<SerialPortInfo> SerialPort::list() {
    std::vector<SerialPortInfo> out;
    std::error_code ec;
    // persistent names (contain the USB product), keyed by the real device
    std::map<std::string, std::string> byId;
    for (const auto& e : fs::directory_iterator("/dev/serial/by-id", ec)) {
        const fs::path real = fs::canonical(e.path(), ec);
        if (!ec) byId[real.string()] = e.path().filename().string();
    }
    for (const auto& e : fs::directory_iterator("/dev", ec)) {
        const std::string name = e.path().filename().string();
        if (name.rfind("ttyACM", 0) != 0 && name.rfind("ttyUSB", 0) != 0) continue;
        SerialPortInfo info;
        info.path = e.path().string();
        // USB product / manufacturer from sysfs
        const fs::path dev = fs::path("/sys/class/tty") / name / "device";
        std::string product, manufacturer;
        for (const fs::path& up : {dev / ".." / "product", dev / ".." / ".." / "product"})
            if (product.empty()) product = readFirstLine(up);
        for (const fs::path& up : {dev / ".." / "manufacturer", dev / ".." / ".." / "manufacturer"})
            if (manufacturer.empty()) manufacturer = readFirstLine(up);
        std::string desc = product.empty() ? std::string(name.rfind("ttyACM", 0) == 0 ? "USB CDC cihazı" : "USB-seri dönüştürücü") : product;
        if (!manufacturer.empty() && desc.find(manufacturer) == std::string::npos) desc = manufacturer + " " + desc;
        auto it = byId.find(info.path);
        if (it != byId.end()) desc += "  (" + it->second + ")";
        info.description = desc;
        std::string low = desc;
        std::transform(low.begin(), low.end(), low.begin(), [](unsigned char c) { return char(std::tolower(c)); });
        // genuine boards report "Arduino"; clones mostly use CH340 / FTDI / CP210x adapters
        info.likelyArduino = low.find("arduino") != std::string::npos || low.find("ch340") != std::string::npos ||
                             low.find("ch341") != std::string::npos || low.find("1a86") != std::string::npos ||
                             name.rfind("ttyACM", 0) == 0;
        out.push_back(info);
    }
    if (fs::exists("/tmp/pegasus-arduino-sim", ec))
        out.push_back({"/tmp/pegasus-arduino-sim", "Pegasus Arduino simülatörü (pegasus_arduino_sim)", true});
    std::stable_sort(out.begin(), out.end(), [](const SerialPortInfo& a, const SerialPortInfo& b) {
        if (a.likelyArduino != b.likelyArduino) return a.likelyArduino;
        return a.path < b.path;
    });
    return out;
}

bool SerialPort::open(const std::string& path, int baud, std::string& err) {
    close();
    const speed_t speed = baudConstant(baud);
    if (!speed) {
        err = "Desteklenmeyen baud hızı: " + std::to_string(baud);
        return false;
    }
    const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        const int e = errno;
        if (e == EACCES || e == EPERM)
            err = "İzin yok: " + path + " açılamadı. Kullanıcınız 'dialout' grubunda değil. Terminalde\n"
                  "  sudo usermod -aG dialout $USER\n"
                  "çalıştırıp oturumu kapatıp açın (ya da bilgisayarı yeniden başlatın).";
        else if (e == ENOENT || e == ENODEV || e == ENXIO)
            err = "Port bulunamadı: " + path + " (kablo takılı mı? 'Yenile' ile listeyi güncelleyin).";
        else if (e == EBUSY)
            err = "Port meşgul: " + path + " başka bir program tarafından kullanılıyor (Arduino IDE seri monitörü açık olabilir).";
        else
            err = "Port açılamadı: " + path + " (" + std::strerror(e) + ")";
        return false;
    }
    // exclusive use: a second program (e.g. a serial monitor) cannot open it while we are connected
    if (ioctl(fd, TIOCEXCL) != 0 && errno != ENOTTY) {
        // not fatal
    }
    struct termios t;
    if (tcgetattr(fd, &t) != 0) {
        err = "Seri port ayarlanamadı (" + std::string(std::strerror(errno)) + "): " + path + " bir seri cihaz mı?";
        ::close(fd);
        return false;
    }
    cfmakeraw(&t);
    cfsetispeed(&t, speed);
    cfsetospeed(&t, speed);
    t.c_cflag |= CLOCAL | CREAD | HUPCL; // HUPCL: DTR drops on close (board stops cleanly)
    t.c_cflag &= ~CRTSCTS;
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &t) != 0) {
        err = "Seri port ayarlanamadı: " + std::string(std::strerror(errno));
        ::close(fd);
        return false;
    }
    tcflush(fd, TCIOFLUSH);
    fd_ = fd;
    path_ = path;
    return true;
}

bool SerialPort::isOpen() const { return fd_ >= 0; }

void SerialPort::close() {
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
}

int SerialPort::read(char* buf, int size) {
    if (fd_ < 0) return -1;
    const ssize_t n = ::read(fd_, buf, size_t(size));
    if (n > 0) return int(n);
    if (n == 0) {
        // a tty returns 0 with VMIN=0; check whether the device is still there
        struct pollfd p{fd_, POLLIN, 0};
        if (poll(&p, 1, 0) > 0 && (p.revents & (POLLHUP | POLLERR | POLLNVAL))) return -1;
        return 0;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return 0;
    return -1; // EIO / ENXIO: unplugged
}

bool SerialPort::write(const std::string& data) {
    if (fd_ < 0) return false;
    size_t done = 0;
    int spins = 0;
    while (done < data.size()) {
        const ssize_t n = ::write(fd_, data.data() + done, data.size() - done);
        if (n > 0) {
            done += size_t(n);
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EINTR) && ++spins < 200) {
            struct pollfd p{fd_, POLLOUT, 0};
            poll(&p, 1, 5);
            continue;
        }
        return false;
    }
    return true;
}

#else // _WIN32 -----------------------------------------------------------------------------------

namespace {
std::wstring widen(const std::string& s) {
    if (s.empty()) return {};
    const int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), int(s.size()), nullptr, 0);
    std::wstring w(size_t(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.data(), int(s.size()), w.data(), n);
    return w;
}
std::string narrow(const wchar_t* w) {
    const int n = WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
    std::string s(size_t(n > 0 ? n - 1 : 0), '\0');
    if (n > 1) WideCharToMultiByte(CP_UTF8, 0, w, -1, s.data(), n, nullptr, nullptr);
    return s;
}
std::string lastErrorText(DWORD e) {
    wchar_t* buf = nullptr;
    FormatMessageW(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, nullptr, e, 0,
                   reinterpret_cast<wchar_t*>(&buf), 0, nullptr);
    std::string s = buf ? narrow(buf) : std::to_string(e);
    if (buf) LocalFree(buf);
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' ')) s.pop_back();
    return s;
}
} // namespace

std::vector<SerialPortInfo> SerialPort::list() {
    // HKLM\HARDWARE\DEVICEMAP\SERIALCOMM: value name = driver device, data = "COMn"
    std::vector<SerialPortInfo> out;
    HKEY key;
    if (RegOpenKeyExW(HKEY_LOCAL_MACHINE, L"HARDWARE\\DEVICEMAP\\SERIALCOMM", 0, KEY_READ, &key) == ERROR_SUCCESS) {
        for (DWORD i = 0;; ++i) {
            wchar_t name[256], data[256];
            DWORD nameLen = 256, dataLen = sizeof(data), type = 0;
            if (RegEnumValueW(key, i, name, &nameLen, nullptr, &type, reinterpret_cast<BYTE*>(data), &dataLen) != ERROR_SUCCESS) break;
            if (type != REG_SZ) continue;
            SerialPortInfo info;
            info.path = narrow(data);
            const std::string dev = narrow(name);
            std::string low = dev;
            std::transform(low.begin(), low.end(), low.begin(), [](unsigned char c) { return char(std::tolower(c)); });
            // usbser = USB CDC (genuine Arduino Uno), ch341/ch340 = most clones, VCP = FTDI / CP210x
            if (low.find("usbser") != std::string::npos) info.description = "USB CDC cihazı (Arduino olabilir)  (" + dev + ")";
            else if (low.find("ch34") != std::string::npos) info.description = "CH340 USB-seri (Arduino klonu olabilir)  (" + dev + ")";
            else if (low.find("vcp") != std::string::npos || low.find("silab") != std::string::npos) info.description = "USB-seri dönüştürücü  (" + dev + ")";
            else info.description = dev;
            info.likelyArduino = low.find("usbser") != std::string::npos || low.find("ch34") != std::string::npos;
            out.push_back(info);
        }
        RegCloseKey(key);
    }
    std::stable_sort(out.begin(), out.end(), [](const SerialPortInfo& a, const SerialPortInfo& b) {
        if (a.likelyArduino != b.likelyArduino) return a.likelyArduino;
        return a.path < b.path;
    });
    return out;
}

bool SerialPort::isOpen() const { return handle_ != nullptr; }

bool SerialPort::open(const std::string& path, int baud, std::string& err) {
    close();
    std::string dev = path;
    if (dev.rfind("\\\\.\\", 0) != 0) dev = "\\\\.\\" + dev; // COM10 and above need the device namespace
    HANDLE h = CreateFileW(widen(dev).c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        const DWORD e = GetLastError();
        if (e == ERROR_FILE_NOT_FOUND || e == ERROR_PATH_NOT_FOUND)
            err = "Port bulunamadı: " + path + " (kablo takılı mı? 'Yenile' ile listeyi güncelleyin).";
        else if (e == ERROR_ACCESS_DENIED || e == ERROR_SHARING_VIOLATION)
            err = "Port meşgul: " + path + " başka bir program tarafından kullanılıyor (Arduino IDE seri monitörü açık olabilir).";
        else
            err = "Port açılamadı: " + path + " (" + lastErrorText(e) + ")";
        return false;
    }
    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(h, &dcb)) {
        err = "Seri port ayarlanamadı: " + lastErrorText(GetLastError()) + " (" + path + " bir seri cihaz mı?)";
        CloseHandle(h);
        return false;
    }
    dcb.BaudRate = DWORD(baud);
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE; // like Linux: raising DTR resets the Uno (it then prints READY)
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    dcb.fOutX = dcb.fInX = FALSE;
    dcb.fNull = FALSE;
    dcb.fAbortOnError = FALSE;
    if (!SetCommState(h, &dcb)) {
        err = "Desteklenmeyen baud hızı ya da port ayarı: " + std::to_string(baud) + " (" + lastErrorText(GetLastError()) + ")";
        CloseHandle(h);
        return false;
    }
    COMMTIMEOUTS to{};
    to.ReadIntervalTimeout = MAXDWORD; // ReadFile returns at once with what is there
    to.ReadTotalTimeoutConstant = 0;
    to.ReadTotalTimeoutMultiplier = 0;
    to.WriteTotalTimeoutConstant = 500;
    to.WriteTotalTimeoutMultiplier = 0;
    SetCommTimeouts(h, &to);
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    handle_ = h;
    path_ = path;
    return true;
}

void SerialPort::close() {
    if (handle_) CloseHandle(static_cast<HANDLE>(handle_));
    handle_ = nullptr;
}

int SerialPort::read(char* buf, int size) {
    if (!handle_) return -1;
    DWORD n = 0;
    if (!ReadFile(static_cast<HANDLE>(handle_), buf, DWORD(size), &n, nullptr)) {
        DWORD errors = 0;
        COMSTAT cs{};
        // a transient line error is cleared and ignored; anything else means the device is gone
        if (GetLastError() == ERROR_OPERATION_ABORTED && ClearCommError(static_cast<HANDLE>(handle_), &errors, &cs)) return 0;
        return -1;
    }
    if (n == 0) {
        // nothing to read: make sure the device is still there (unplugged USB fails here)
        DWORD errors = 0;
        COMSTAT cs{};
        if (!ClearCommError(static_cast<HANDLE>(handle_), &errors, &cs)) return -1;
    }
    return int(n);
}

bool SerialPort::write(const std::string& data) {
    if (!handle_) return false;
    size_t done = 0;
    while (done < data.size()) {
        DWORD n = 0;
        if (!WriteFile(static_cast<HANDLE>(handle_), data.data() + done, DWORD(data.size() - done), &n, nullptr)) return false;
        if (n == 0) return false; // write timeout
        done += n;
    }
    return true;
}

#endif // _WIN32

} // namespace pegasus
