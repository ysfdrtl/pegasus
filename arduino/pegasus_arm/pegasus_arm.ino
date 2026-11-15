/*
  Pegasus robot kol firmware'i (Arduino Uno)

  Bilgisayardaki Pegasus uygulaması ile USB seri port üzerinden haberleşir (Bluetooth / telefon
  kontrolü yok). Bağlantı şeması: "Arduino-Robot-Arm-Schematic-Circuit-Diagram.png" — servolar harici
  5V kaynaktan beslenir, sinyal uçları:

    kanal 1  Bel (Waist)          MG996R  -> D5
    kanal 2  Omuz (Shoulder)      MG996R  -> D6
    kanal 3  Dirsek (Elbow)       MG996R  -> D7
    kanal 4  Bilek dönüş (Roll)   SG90    -> D8
    kanal 5  Bilek eğim (Pitch)   SG90    -> D9
    kanal 6  Kıskaç (Gripper)     SG90    -> D10

  (Şemadaki HC-05 modülü artık gerekmez; D3/D4 boş kalır.)

  Protokol: 115200 baud, satır tabanlı ASCII ('\n' ile biter, '\r' yok sayılır). Açılar servo
  derecesinin 10 katıdır (0..1800 = 0..180.0°), böylece 0.1° çözünürlük elde edilir.

    Bilgisayar -> Arduino                         Arduino -> Bilgisayar
    PING                                          PONG
    INFO                                          INFO pegasus-arm <sürüm> ch=6 pins=5,6,7,8,9,10
    M <seq> <süre_ms> <a1> .. <a6>                A <seq> <kuyruktaki_boş_yer>   (hareket kuyruğa alındı)
    S <kanal 1-6> <a>                             OK      (tek servo, hız sınırıyla)
    V <derece/saniye>                             OK      (en yüksek servo hızı, 1..600)
    L <kanal> <min> <max>                         OK      (kanal sınırları, 0..1800)
    HOME                                          OK      (başlangıç pozu; servoları ilk kez sürer)
    STOP                                          OK      (kuyruğu boşalt, olduğu yerde dur)
    ATTACH / DETACH                               OK      (servoları sür / serbest bırak)
    GET                                           POS <hareket 0/1> <a1> .. <a6> <kuyruktaki_boş_yer> <homed 0/1>
    (açılışta)                                    READY pegasus-arm <sürüm> servolar serbest
    (hata)                                        ERR <kod> <açıklama>

  "M" komutları bir kuyruğa (8 adım) girer; her adım, mevcut konumdan hedefe <süre_ms> içinde
  doğrusal olarak gider. Süre, hız sınırını aşmayacak şekilde gerekirse uzatılır. Bilgisayar
  "A" cevabındaki boş yer sayısına bakarak kuyruğu taşırmadan akış (streaming) yapar.

  Açılış: kart açıldığında (ve bilgisayar portu açıp kartı yeniden başlattığında) servolara HİÇ darbe
  gönderilmez; servolar serbesttir ve kol hareket etmez. Kol ancak elle verilen "HOME" komutuyla
  başlangıç pozuna alınır. HOME verilmeden M / S / ATTACH reddedilir (ERR 6). Kartın servoların gerçek
  konumunu bilmesinin bir yolu olmadığından ilk HOME'da servolar başlangıç pozuna kendi hızlarıyla
  gider: HOME'dan önce kolu elle başlangıç pozuna yakın getirin. DETACH'tan sonra kol elle
  oynatılabileceği için yeniden HOME gerekir.

  Güvenlik: "M" ile akış başladıktan sonra bilgisayar 1.5 s boyunca hiç komut göndermezse (uygulama
  kapandı, kablo çekildi) kuyruk boşaltılır ve kol olduğu yerde durur. Uygulama bağlıyken her 0.5 s'de
  bir PING gönderir. Seri monitörden elle verilen S / HOME komutları bu süre sınırına tabi değildir.
*/

#include <Servo.h>

#define PEGASUS_FW_VERSION "1.1"

const uint8_t NCH = 6;
const uint8_t PINS[NCH] = {5, 6, 7, 8, 9, 10};
// Örnek koddaki başlangıç pozu (derece x10)
const int16_t HOME_POS[NCH] = {900, 1500, 350, 1400, 850, 800};

const uint8_t QUEUE_LEN = 8;
const unsigned long WATCHDOG_MS = 1500;
const unsigned long UPDATE_MS = 10; // servo güncelleme periyodu (100 Hz)

struct Step {
  int16_t target[NCH];
  uint16_t durationMs;
};

Servo servos[NCH];
int16_t minPos[NCH], maxPos[NCH];
float current[NCH];   // anlık konum (derece x10)
float startPos[NCH];  // aktif adımın başlangıcı
float goal[NCH];      // aktif adımın hedefi
unsigned long stepStart = 0, stepDuration = 0;
bool stepActive = false;
float maxSpeed = 90.0f; // derece / saniye
bool attached = false;
bool homed = false; // HOME verildi mi (konum biliniyor ve servolar sürülüyor)

Step queue[QUEUE_LEN];
uint8_t qHead = 0, qCount = 0;

char line[96];
uint8_t lineLen = 0;
bool lineOverflow = false;
unsigned long lastCommand = 0, lastUpdate = 0;
bool watchdogArmed = false; // yalnızca bilgisayardan akış (M) yapılırken

// ---------------------------------------------------------------------------------------------
// servo çıkışı

int clampPos(uint8_t ch, long v) {
  if (v < minPos[ch]) return minPos[ch];
  if (v > maxPos[ch]) return maxPos[ch];
  return (int)v;
}

void writeServo(uint8_t ch, float pos10) {
  // Servo kütüphanesinin varsayılan darbe aralığı: 544..2400 us = 0..180°
  const long us = 544L + (long)(pos10 * (2400.0f - 544.0f) / 1800.0f + 0.5f);
  if (attached) servos[ch].writeMicroseconds((int)us);
}

void attachAll() {
  if (attached) return;
  for (uint8_t i = 0; i < NCH; i++) {
    // ilk darbe mevcut konumla başlasın (servo zıplamasın)
    servos[i].attach(PINS[i]);
  }
  attached = true;
  for (uint8_t i = 0; i < NCH; i++) writeServo(i, current[i]);
}

void detachAll() {
  for (uint8_t i = 0; i < NCH; i++) servos[i].detach();
  attached = false;
}

// ---------------------------------------------------------------------------------------------
// hareket

// Mevcut konumdan hedefe; süre hız sınırına göre gerekirse uzatılır.
void beginStep(const int16_t* target, unsigned long durationMs) {
  float maxDelta = 0.0f;
  for (uint8_t i = 0; i < NCH; i++) {
    startPos[i] = current[i];
    goal[i] = clampPos(i, target[i]);
    const float d = fabs(goal[i] - startPos[i]);
    if (d > maxDelta) maxDelta = d;
  }
  const unsigned long minDuration = (unsigned long)(maxDelta * 100.0f / maxSpeed); // (x10 derece) / (derece/s) -> ms
  stepDuration = durationMs > minDuration ? durationMs : minDuration;
  stepStart = millis();
  stepActive = true;
}

void stopMotion() {
  watchdogArmed = false;
  qCount = 0;
  stepActive = false;
  for (uint8_t i = 0; i < NCH; i++) goal[i] = startPos[i] = current[i];
}

void updateMotion() {
  const unsigned long now = millis();
  if (!stepActive && qCount > 0) {
    const Step& s = queue[qHead];
    beginStep(s.target, s.durationMs);
    qHead = (qHead + 1) % QUEUE_LEN;
    qCount--;
  }
  while (stepActive) {
    const unsigned long elapsed = now - stepStart;
    if (elapsed < stepDuration) {
      const float f = (float)elapsed / (float)stepDuration;
      for (uint8_t i = 0; i < NCH; i++) current[i] = startPos[i] + (goal[i] - startPos[i]) * f;
      break;
    }
    // adım bitti: bir sonraki adım tam bitiş anından başlar (akışta boşluk / gecikme birikmez)
    for (uint8_t i = 0; i < NCH; i++) current[i] = goal[i];
    stepActive = false;
    if (qCount > 0) {
      const unsigned long endTime = stepStart + stepDuration;
      const Step& s = queue[qHead];
      beginStep(s.target, s.durationMs);
      stepStart = endTime;
      qHead = (qHead + 1) % QUEUE_LEN;
      qCount--;
    }
  }
  for (uint8_t i = 0; i < NCH; i++) writeServo(i, current[i]);
}

bool pushStep(const int16_t* target, uint16_t durationMs) {
  if (qCount >= QUEUE_LEN) return false;
  Step& s = queue[(qHead + qCount) % QUEUE_LEN];
  for (uint8_t i = 0; i < NCH; i++) s.target[i] = target[i];
  s.durationMs = durationMs;
  qCount++;
  return true;
}

// ---------------------------------------------------------------------------------------------
// komutlar

// Boşlukla ayrılmış tamsayıları okur; okunan sayı adedini döndürür.
uint8_t parseInts(const char* p, long* out, uint8_t maxCount) {
  uint8_t n = 0;
  while (*p && n < maxCount) {
    while (*p == ' ') p++;
    if (!*p) break;
    char* end;
    const long v = strtol(p, &end, 10);
    if (end == p) return 255; // sayı değil
    out[n++] = v;
    p = end;
  }
  while (*p == ' ') p++;
  return *p ? 255 : n;
}

void reply(const char* s) { Serial.println(s); }

void printPositions(const char* tag) {
  Serial.print(tag);
  Serial.print(stepActive || qCount > 0 ? " 1" : " 0");
  for (uint8_t i = 0; i < NCH; i++) {
    Serial.print(' ');
    Serial.print((int)(current[i] + 0.5f));
  }
  Serial.print(' ');
  Serial.print((int)(QUEUE_LEN - qCount));
  Serial.println(homed ? " 1" : " 0");
}

// HOME verilmeden servolar sürülmez: konum bilinmiyor
bool requireHomed() {
  if (homed) return true;
  reply("ERR 6 once HOME komutu verin (servolar serbest)");
  return false;
}

bool startsWith(const char* s, const char* cmd) {
  while (*cmd) {
    if (*s++ != *cmd++) return false;
  }
  return *s == 0 || *s == ' ';
}

void handleLine(char* s) {
  long v[NCH + 2];
  if (startsWith(s, "PING")) {
    reply("PONG");
  } else if (startsWith(s, "INFO")) {
    Serial.print("INFO pegasus-arm " PEGASUS_FW_VERSION " ch=6 pins=5,6,7,8,9,10 homed=");
    Serial.println(homed ? 1 : 0);
  } else if (startsWith(s, "M")) {
    if (!requireHomed()) return;
    if (parseInts(s + 1, v, NCH + 2) != NCH + 2) {
      reply("ERR 2 M: seq sure a1..a6 bekleniyor");
      return;
    }
    int16_t t[NCH];
    for (uint8_t i = 0; i < NCH; i++) t[i] = (int16_t)clampPos(i, v[2 + i]);
    const long dur = v[1] < 0 ? 0 : (v[1] > 60000 ? 60000 : v[1]);
    if (!pushStep(t, (uint16_t)dur)) {
      Serial.print("ERR 3 kuyruk dolu ");
      Serial.println(v[0]);
      return;
    }
    if (!attached) attachAll();
    watchdogArmed = true;
    Serial.print("A ");
    Serial.print(v[0]);
    Serial.print(' ');
    Serial.println(QUEUE_LEN - qCount);
  } else if (startsWith(s, "S")) {
    if (!requireHomed()) return;
    if (parseInts(s + 1, v, 2) != 2 || v[0] < 1 || v[0] > NCH) {
      reply("ERR 2 S: kanal(1-6) aci bekleniyor");
      return;
    }
    // diğer kanallar yerinde kalır
    stopMotion();
    int16_t t[NCH];
    for (uint8_t i = 0; i < NCH; i++) t[i] = (int16_t)(current[i] + 0.5f);
    t[v[0] - 1] = (int16_t)clampPos(v[0] - 1, v[1]);
    pushStep(t, 0);
    if (!attached) attachAll();
    reply("OK");
  } else if (startsWith(s, "V")) {
    if (parseInts(s + 1, v, 1) != 1 || v[0] < 1 || v[0] > 600) {
      reply("ERR 2 V: 1..600 derece/s");
      return;
    }
    maxSpeed = (float)v[0];
    reply("OK");
  } else if (startsWith(s, "L")) {
    if (parseInts(s + 1, v, 3) != 3 || v[0] < 1 || v[0] > NCH || v[1] < 0 || v[2] > 1800 || v[1] > v[2]) {
      reply("ERR 2 L: kanal min max (0..1800)");
      return;
    }
    minPos[v[0] - 1] = (int16_t)v[1];
    maxPos[v[0] - 1] = (int16_t)v[2];
    reply("OK");
  } else if (startsWith(s, "HOME")) {
    stopMotion();
    if (!homed) {
      // konum bilinmiyor: servolar başlangıç pozuna doğrudan (kendi hızlarıyla) gider
      for (uint8_t i = 0; i < NCH; i++) current[i] = startPos[i] = goal[i] = clampPos(i, HOME_POS[i]);
      attachAll();
      homed = true;
    } else {
      pushStep(HOME_POS, 0); // konum biliniyor: hız sınırıyla
    }
    reply("OK");
  } else if (startsWith(s, "STOP")) {
    stopMotion();
    reply("OK");
  } else if (startsWith(s, "ATTACH")) {
    if (!requireHomed()) return;
    attachAll();
    reply("OK");
  } else if (startsWith(s, "DETACH")) {
    stopMotion();
    detachAll();
    homed = false; // kol elle oynatılabilir: yeniden HOME gerekir
    reply("OK");
  } else if (startsWith(s, "GET")) {
    printPositions("POS");
  } else if (s[0] == 0) {
    // boş satır
  } else {
    reply("ERR 1 bilinmeyen komut");
  }
}

void readSerial() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[lineLen] = 0;
      lastCommand = millis();
      if (lineOverflow) reply("ERR 4 satir cok uzun");
      else handleLine(line);
      lineLen = 0;
      lineOverflow = false;
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    } else {
      lineOverflow = true;
    }
  }
}

// ---------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < NCH; i++) {
    minPos[i] = 0;
    maxPos[i] = 1800;
    current[i] = startPos[i] = goal[i] = HOME_POS[i];
  }
  // servolar serbest: HOME komutu gelene kadar hiçbir pine darbe gönderilmez
  attached = false;
  homed = false;
  lastCommand = millis();
  reply("READY pegasus-arm " PEGASUS_FW_VERSION " servolar serbest");
}

void loop() {
  readSerial();
  const unsigned long now = millis();
  if (now - lastUpdate >= UPDATE_MS) {
    lastUpdate = now;
    updateMotion();
  }
  // bağlantı sessiz kaldıysa yeni hareket yapma
  if (watchdogArmed && (stepActive || qCount > 0) && now - lastCommand > WATCHDOG_MS) {
    stopMotion();
    reply("ERR 5 baglanti zaman asimi, durduruldu");
    lastCommand = now;
  }
}
