#include "hw/SerialPort.h"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace fs = std::filesystem;

namespace pegasus {

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

} // namespace pegasus
