# Pegasus

Servo motorlu robot kol için C++ / Vulkan masaüstü uygulaması. Bu aşamada **görüntü / çalışma alanı** kısmı hazır:
STL yükleme, nokta-kenar-yüzey seçimi, nesneleri serbestçe taşıma ve çoklu nesne çarpışma kontrolü.
Robot kol yüklenip eklemleri otomatik algılanır; ileri kinematik ile elle, **ters kinematik (IK)** ile de
kütüphanedeki hareket formüllerine göre seçilen kenarlar boyunca otomatik sürülür.
Arduino (seri port) kısmı sonraki aşamada eklenecek.

## Derleme

Gerekenler: CMake ≥ 3.20, C++20 derleyici, Vulkan SDK/başlıkları + `glslc`, GLFW 3.3+ (yoksa otomatik indirilir).
GLM, Dear ImGui (docking) ve ImGuizmo ilk yapılandırmada otomatik indirilir (internet gerekir).

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/pegasus                 # boş sahne
./build/pegasus parca1.stl parca2.stl
```

Ortam değişkenleri:
- `PEGASUS_GPU=intel` / `PEGASUS_GPU=nvidia` — GPU seçimi (ad içinde arama). Varsayılan: harici GPU.
- `PEGASUS_VALIDATION=1` — Vulkan validation katmanını açar.

## Kullanım

STL dosyası eklemek: dosyayı pencereye sürükleyip bırakın, **Dosya > STL Aç** (Ctrl+O) ya da komut satırı.
Binary ve ASCII STL desteklenir. Birim mm, Z ekseni yukarı.

| Kısayol | İşlev |
|---|---|
| Q / 1 / 2 / 3 | Nesne / Nokta / Kenar / Yüzey aracı |
| W / E / R, L | Gizmo: taşı / döndür / ölçekle, dünya-yerel eksen |
| Sol sürükle (Nesne aracı) | Nesneyi zemin düzleminde taşı; Shift ile dikey (Z) |
| Ctrl (sürüklerken) | Adımlı (snap) hareket |
| Shift + tık | Seçime ekle / çıkar |
| Sağ/orta sürükle (veya Alt+sol boşlukta) | Kamerayı döndür; Shift ile kaydır; teker: yakınlaş |
| F / Home | Seçili nesneye / tüm sahneye odaklan |
| G, Ctrl+D, Delete | Zemine indir, çoğalt, sil |
| F1 | Kısayol listesi |

**Seçim modları**
- *Nokta*: ağdaki köşe noktaları (arkada kalan noktalar seçilmez). İki nokta arası mesafe/ağırlık merkezi gösterilir.
- *Kenar*: "Keskin kenarlar" (açısı eşik değerin üstündeki kenarlar) veya "Tüm kenarlar" (üçgen ağı). Uzunluk/açı ölçümü.
- *Yüzey*: "Tek üçgen", "Düzlemsel yüzey" (aynı düzlemdeki bitişik üçgenler) veya "Yumuşak yüzey"
  (keskin kenarla ayrılmamış bölge, ör. silindirin yan yüzü). Alan, normal, iki yüzey arası açı/mesafe.

**Yüzey seçim yardımcıları (Blender benzeri, Yüzey aracında)**

| Kısayol | İşlev |
|---|---|
| L / Shift+L | İmlecin altındaki bağlı yüzeyi seç / çıkar (ör. bir kürenin tamamı tek tuşla) |
| Ctrl+L | Mevcut seçimi bağlı yüzeye genişlet |
| Ctrl + / Ctrl - | Seçimi bir halka komşu üçgen büyüt / küçült |
| Alt + tık | Döngü seçimi: tıklanan kenarı kesen yüz halkası (Shift ile ekle) |
| C | Fırça seçimi: sol sürükle = ekle, Shift + sürükle = çıkar, teker = fırça boyutu, C/Esc = çık |
| A, Ctrl+I | Nesnenin tüm yüzeyi, seçimi ters çevir |

- *Sınır açısı* (Araçlar paneli): bağlı seçim bu açıdan keskin kenarlarda durur; 180° = bağlı her şey.
- Bu araçlar üçgen düzeyinde çalışır; bölge seçimi (düzlemsel/yumuşak) otomatik olarak üçgenlere çevrilir ve
  Shift + tık ile bölgeler eklenip çıkarılabilir.
- STL dosyaları üçgenlerden oluştuğu için döngü seçiminde üçgen çiftleri (köşegeni paylaşan, aynı düzleme yakın)
  yeniden dörtgen sayılır; halka bir üçgene (ör. küre kutbu) ya da açık kenara gelince durur.

**Çarpışma kontrolü** (sağ panel > Çarpışmalar)
- Tüm görünür nesne çiftleri AABB ile elenir, ardından BVH + üçgen-üçgen (SAT) testiyle kesin kontrol yapılır.
- Bir nesnenin tamamen diğerinin içinde kalması da algılanır (kapalı ağlarda).
- Sadece temas (üst üste konmuş parçalar) çarpışma sayılmaz; aynı yöne bakan çakışık yüzeyler iç içe geçme sayılır.
- Çarpışan nesneler kırmızı görünür, kesişen üçgenler vurgulanır.
- "Çarpışmada hareketi engelle" açıkken, sürüklenen nesne temas noktasında durur.

## Robot kol

**Dosya > Robot Kol Yükle (klasör)...** ya da `./build/pegasus --robot "robot kol stl parçaları"`.
Klasördeki her STL bir parça olmalı ve montaj koordinatlarında dışa aktarılmış olmalı (SolidWorks'te
montajı "her parçayı ayrı dosya" olarak kaydetmek bunu sağlar).

Otomatik eklem algılama:
- Mil, yatak ve pim delikleri dairesel kenar halkalarından bulunur; komşu iki parçada **eş eksenli** daire varsa
  orada bir bağlantı vardır. Tek ortak eksen = döner eklem, iki ve daha fazla ortak eksen = sabit bağlantı,
  büyük ölçüde gövdenin içinde kalan parça (servo) = sabit.
- Hiyerarşi tabandan başlayarak kurulur (Taban → Bel → Kol 1 → Kol 2 → Kol 3 → Kıskaç gövdesi → ...).
- Kapalı zincirler mekanizma olarak çözülür: **paralelkenar bağlantı** (parmaklar dönmeden ötelenir) ve
  **dişli çifti** (ters yönde döner) bağımlı (mimic) eklem olur; kıskaç tek bir açıyla sürülür.
  Sürücü eklem, servo miliyle aynı eksendeki dişlidir.
- Limitler, eklem ±90° taranıp robotun kendisiyle ya da zeminle çarpıştığı ilk açıya göre bulunur.
- TCP (uç nokta) kıskaç parmak uçlarının ortasına konur.

Bu robot için bulunan eklemler: **Bel, Omuz, Dirsek, Bilek dönüş, Bilek eğim, Kıskaç** (şemadaki 6 servo sırası).
0° = STL'deki montaj pozu (kol dik). Servo açısına çevirme (kalibrasyon) Arduino aşamasında eklenecek.

Robot paneli:
- *İleri kinematik*: her eklem için kaydırıcı; TCP konumu (tabana ve dünyaya göre) ve yönelimi (R/P/Y).
- *Eklem düzenleme*: ad, bağlı olduğu parça, tip (sabit / bağımsız / bağımlı), eksen noktası ve yönü,
  limitler, bağımlı eklemler için takip edilen eklem + çarpan + ofset.
  **Seçimden al**: Nokta/Kenar/Yüzey aracıyla seçilen geometriden ekseni hesaplar (bir deliğin silindirik yüzeyi
  → delik ekseni, daire üzerinde 3+ nokta → daire merkezi ve normali, düz yüzey → normali).
- *Kaydet*: tanım `robot.pegasus` olarak STL klasörüne yazılır; sonraki yüklemede algılama yerine bu dosya kullanılır.
- Robot parçası sürüklenince ya da gizmo ile tüm robot taşınır. Robotun kendisiyle ve sahnedeki nesnelerle
  çarpışması kontrol edilir; "Çarpışmada hareketi engelle" açıksa eklem temas noktasında durur.

## Hareket kütüphanesi ve yol takibi (Hareket paneli)

1. **Başlık** seçin (kıskaç, kaynak torcu, kalem, sprey tabancası...). Başlık, takım ucunun robot TCP'sine göre
   ofsetidir; takım ekseni TCP'nin +Z ekseni (kolun uzandığı yön).
2. **Kenar aracı (2)** ile kenar seçip *Seçili kenarlardan yol oluştur*'a basın:
   - 2 kenar (ör. aralarında 0.5 mm boşluk olan iki küpün karşılıklı kenarları): yol iki kenarın **tam ortasından**,
     alttan üste doğru gider. Yüzey normali, iki kenarın ortak çalışma yüzeyinden bulunur.
   - 1 kenar: yol kenar boyunca gider, normal komşu yüzeylerin açıortayıdır.
   Yol parçalara bağlıdır; parçalar taşınınca yol da güncellenir.
3. **Hareket** seçin ve parametreleri değiştirin. Yol, erişilebilirliği ile birlikte anında çizilir
   (yeşil = robot erişebiliyor, kırmızı = erişemiyor).
4. **Çalıştır**: robot yolun başına gider, normal boyunca yaklaşır, yolu izler ve geri çekilir.
   Duraklat/devam, zaman kaydırıcısı, hız çarpanı ve "çarpışmada durdur" seçenekleri vardır.

### Hareket formülü yazmak

Hareketler `library/motions/*.pmotion` dosyalarıdır (uygulamadan da düzenlenip *Kütüphaneye kaydet* ile yazılabilir).
Formüller **yerel dikiş çerçevesinde** takım ucunun ofsetini verir:

| eksen | anlamı |
|---|---|
| `x` | yan (dikişe dik, yüzey üzerinde) |
| `y` | ilerleme yönünde ek ofset |
| `z` | yüzey normali (dışarı doğru, mm) |
| `work_angle`, `travel_angle` | takım eğimi: yana / ilerleme yönüne (derece) |

Değişkenler: `s` (yol boyunca mm), `u` (0..1), `L` (yol uzunluğu), `t` (saniye), `v` (hız mm/s) ve dosyadaki parametreler.
Fonksiyonlar: `sin cos tan asin acos atan atan2 sqrt abs exp log pow min max floor ceil round fract mod clamp sign
step smoothstep lerp if deg rad`, dalga biçimleri `tri(x)` (üçgen), `saw(x)` (testere), `sq(x)` (kare),
`pulse(x, doluluk)`; sabitler `pi tau e`.

```ini
# library/motions/duz.pmotion
name = Optimum noktadan düz ilerle
param h = 1 | 0 | 50 | mm | Yüzeyden yükseklik      # param AD = varsayılan | min | max | birim | etiket
x = 0
y = 0
z = h
```

Örnekler: sinüs `x = A*sin(2*pi*s/W)`, zikzak `x = A*tri(s/W)`, gagalama `z = h + D*(0.5 - 0.5*cos(2*pi*s/P))`,
dairesel örgü `x = R*sin(2*pi*s/P)`, `y = R*(cos(2*pi*s/P) - 1)`.

Başlıklar `library/tools/*.ptool` dosyalarıdır (`name`, `description`, `tip_offset = x y z`, `radius`, `color`).

## Yüzey tarama (sprey boya)

Hareket panelinde **İş tipi: Yüzey tarama** seçin.

1. **Başlık**: ör. *Sprey tabancası*.
2. **Yüzey aracı (3)** ile boyanacak yüzeyi seçin — düz yüzler için *Düzlemsel yüzey*, küre/figür gibi çok sayıda
   küçük poligondan oluşan yüzeyler için **L** (bağlı seçim), **fırça (C)** veya döngü seçimi — ve *Seçili yüzeylerden
   tarama alanı oluştur*'a basın. Seçilen yüzler mavi görünür; parçalar taşınınca tarama yolu da güncellenir.
3. **Desen** seçip parametreleri ayarlayın — yol anında yeniden hesaplanır:

| parametre | anlamı |
|---|---|
| `offset` | Kenardan boşluk: boya noktaları yüzün sınırına (dış kenar, delik kenarı) en az bu kadar uzak kalır |
| `spacing` | İki tarama çizgisi arasındaki mesafe; çizgiler yüzün ortasına göre simetrik dağıtılır |
| `standoff` | Tarama sırasında tabanca ucunun yüzeyden uzaklığı (yüzey normali boyunca) |
| `angle` | Tarama çizgilerinin yönü (0° = yüzün en uzun ekseni) |
| `lift` | Kalkarak geçişlerde (tek yönlü desen, yüzler arası) kalkma yüksekliği |
| `crease` | Kırılma açısı: seçili üçgenler bu açıdan keskin kenarlarda ayrı yüzey parçalarına bölünür (varsayılan 60°) |
| `tilt` | En fazla eğim: kol tabancayı yüzeye dik tutamıyorsa bu açıya kadar eğebilir (varsayılan 30°, 0 = her zaman dik) |

4. **Çalıştır**: robot her yüzü sırayla tarar; boya yaparken püskürtme konisi gösterilir, geçişler gri çizilir
   (takım kapalı).

Desenler `library/scans/*.pscan` dosyalarıdır (hareket dosyalarıyla aynı format + `pattern`, `link`, `orient`):

- `pattern = zigzag` (gidip gelerek), `oneway` (hep aynı yönde, kalkarak döner), `crosshatch` (iki dik yönde)
- `link = surface` (çizgiler arası yüzeye paralel geçiş) veya `lift` (kalkarak geçiş)
- `orient = surface` (takım yerel yüzey normaline dik — eğri yüzlerde yüzeyi takip eder) veya `fixed` (ortalama normal)
- `x / y / z / work_angle / travel_angle` formülleri her tarama çizgisine eklenir; ek değişkenler `k` (çizgi no) ve
  `n` (çizgi sayısı). Örnek: *Dalgalı tarama* `x = A*sin(2*pi*s/W)`.

Hazır desenler: Zikzak tarama, Tek yönlü tarama, Çapraz tarama, Dalgalı tarama.

Yöntem (poligon poligon değil, **bütün yüzey** üzerinden):
- Seçili üçgenler bağlantılarına göre *yüzey parçalarına* ayrılır; yalnızca `crease`'ten keskin kenarlar böler, çok küçük
  parçalar komşusuna katılır. 2208 üçgenli bir küre tek parçadır.
- Her parça paralel düzlemlerle **kesilir**: kesit eğrileri yüzeyin üzerinde kesintisiz tarama çizgileridir. Düzlem
  aralığı eğime göre ayarlanır, böylece komşu çizgiler *yüzey üzerinde* `spacing` kadar aralıklı kalır (kubbenin yan
  tarafında çizgiler sıklaşır, boşluk kalmaz).
- Çizgilerin yalnızca parça sınırına (dış kenar, delik) en az `offset` uzak kısımları boyanır.
- Tabanca yönü köşe normallerinin yumuşatılmış ortalamasıdır; fasetten fasete titremez.
- Kapalı nesnelerde (bütün küre, figür) robota bakan taraf boyanır; arka taraf atlanır ve bildirilir.
- `tilt` > 0 iken IK, Blender'daki gibi iki öncelikli çözülür: birinci görev tabanca ekseninin yüzeydeki hedef noktaya
  tam `standoff` mesafeden nişan alması, ikinci görev (birincinin sıfır uzayında) yüzeye olabildiğince dik durmasıdır.
  5 eksenli kol takım eksenini yalnızca kendi dikey düzleminde çevirebildiği için bu, eğri yüzeylerde erişimi belirgin
  artırır. Erişilemeyen noktalarda ve kolun konfigürasyon değiştirdiği sıçramalarda tabanca kapalıdır.

### IK çözücü (Blender'dan uyarlandı)

`src/robot/IkSolver.cpp`, Blender'ın `intern/iksolver` yapısını izler: eklemler (IK_QSegment) limitli serbestlik
dereceleri, görevler (IK_QTask) konum / takım yönü / tam yönelim; Jacobian SVD ile **SDLS** (seçici sönümlü en küçük kareler)
veya **DLS** ile ters çevrilir; limit aşan eklem kilitlenip çözüm tekrarlanır; ikincil görevler (ör. tercih edilen poz)
birincil görevin sıfır uzayına izdüşürülür. Robot 5 serbestlik dereceli olduğundan görev "konum + takım yönü"dür
(takım kendi ekseni etrafında serbest). Yol takibinde her nokta bir öncekinin çözümünden başlar; başarısız olursa
DLS ve çoklu başlangıç noktası denenir, mevcut poza en yakın çözüm seçilir. Canlı önizlemede IK kontrolü ~200 ms ile
sınırlıdır (kontrol edilmeyen noktalar mavi görünür); *Çalıştır* tüm noktaları çözer.

### Kol neden bazen "baştan aşağı" yön değiştiriyor?

Kısa cevap: aynı takım pozuna kolun birden fazla eklem açısı kombinasyonuyla (IK *dalı*) ulaşılabiliyor ve servoların
açı sınırları bir dalda kalmayı her zaman mümkün kılmıyor. Ayrıntılar:

1. **Bilek dönüşü ±90° ile sınırlı.** Takım ekseninin yönü, *bilek dönüş* ve *bilek eğim* açılarının birlikte
   sonucudur; (dönüş θ, eğim φ) ile (dönüş θ ± 180°, eğim −φ) aynı takım yönünü verir. Yol boyunca gereken dönüş
   +90°'yi aşınca tek çıkış, bileği −90° tarafına çevirip eğimi ters işaretle kullanmaktır: iki komşu nokta arasında
   bilek 180° döner, kol tamamen farklı bir duruşa geçer. Bel ekseni de ±90° ile sınırlı olduğundan robotun "arkasına"
   bakan pozlarda benzer bir dal değişimi olur.
2. **Kol 5 eksenli.** Takım ekseni her zaman kolun dikey düzleminde kalır; bazı yüzey yönlerine hiç ulaşılamaz
   (`tilt` bunu yumuşatır ama kaldırmaz). Erişilemeyen noktada IK çözücü "en iyi denemesini" verir, bu da çoğu zaman
   bambaşka bir dalda, anlamsız bir pozdur.
3. **IK yereldir (Blender'daki gibi).** Her nokta bir öncekinin çözümünden başlar. Mevcut dalda bir sonraki noktaya
   ulaşılamazsa çözücü başka başlangıç noktaları dener ve bulduğu ilk iyi çözümler arasından mevcut poza en yakınını
   seçer — ama bu yine de başka bir dal olabilir.
4. **Yörünge eklem uzayında aradeğerlenir.** İki anahtar poz arasında her eklem doğrusal hareket eder; iki poz farklı
   dallardaysa takım ucu bu geçişte düz bir çizgi izlemez, geniş bir yay çizer (eskiden parçaların içinden geçebiliyordu).

Yapılan iyileştirmeler (`src/motion/PathPlanner.cpp`):
- **Dal onarımı (`repairJumps`)**: bir sıçrama bulunduğunda, sıçramadan önceki erişilebilir bölüm yeni daldan geriye
  doğru yeniden çözülür. Böylece dal değişimi, bölümün başına (takımın zaten kapalı olduğu geçişe) taşınır.
- **Erişilemeyen pozlar kullanılmaz**: çözülemeyen noktaların anlamsız eklem açıları yörüngeye konmaz; kol son ulaşılan
  pozdan bir sonrakine takım kapalıyken geçer.
- **Sıçramada takım kapalı ve geri çekilmiş**: iki komşu nokta arasında herhangi bir eklem 10°'den fazla değişiyorsa
  tabanca kapatılır, takım önce kendi ekseni boyunca geri çekilir, dönüş orada yapılır, sonra geri yaklaşılır.
- **Çarpışmasız geçişler**: takım kapalıyken yapılan her hareket (yüzler arası geçiş, dal değişimi) sahnedeki parçalara
  karşı ince adımlarla kontrol edilir; uç bir parçanın içine girecekse geri çekilerek, kaldırılarak ya da yüksek bir ara
  noktadan dolaşılarak yeniden yönlendirilir. Bulunamazsa uyarı verilir. Tarama planındaki geçiş çizgileri de parçalara
  `5 mm` pay bırakacak şekilde dolaştırılır (ör. küpün iki yan yüzü arasında köşenin etrafından).
- Ölçüm (küp + iki bitişik yüz, 24 farklı yerleşim): takım ucu artık hiçbir yerleşimde küpün içine girmiyor; takım
  açıkken 90°'den büyük dönüş yok. Kalan büyük dönüşler (bilek sınırı nedeniyle kaçınılmaz olanlar) takım kapalı ve
  geri çekilmişken yapılıyor.

Daha az dönüş için: parçayı robotun önüne, tabancanın robottan *dışarı* baktığı tarafa yerleştirin; yüzeyleri
robota çevirin; `angle` ile tarama yönünü değiştirin (çizgiler bilek dönüşünü daha az zorlayabilir); gerekirse `tilt`'i
azaltın. Bunu tamamen ortadan kaldırmak için daha geniş açılı (±180°) bir bilek servosu veya 6 eksenli bir kol gerekir.

Not: çarpışma kontrolü takım **ucunu** (püskürtme noktası) dikkate alır; kolun gövdesi ile parçalar arasındaki temas
çalıştırma sırasında ayrıca kontrol edilir ("Çarpışmada durdur"). Takım başlığının kendi gövdesi henüz modellenmedi.

## Arduino ile gerçek kolu sürmek

Kontrol tamamen bilgisayardan, USB kablosu üzerinden yapılır (örnek koddaki HC-05 / telefon kısmı kaldırıldı).

### 1. Firmware'i karta yükleyin
- Arduino IDE ile `arduino/pegasus_arm/pegasus_arm.ino` dosyasını açın, kart: **Arduino Uno**, doğru portu seçip
  **Yükle**. (Ek kütüphane gerekmez; yalnızca IDE ile gelen `Servo`.) Komut satırından:
  `arduino-cli compile --fqbn arduino:avr:uno arduino/pegasus_arm && arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno arduino/pegasus_arm`
- Bağlantı şeması değişmedi (`Arduino-Robot-Arm-Schematic-Circuit-Diagram.png`); HC-05 modülü ve dirençleri artık
  gerekmez. Servolar **harici 5V** kaynaktan beslenmeli, kaynağın GND'si Arduino GND'sine bağlı olmalı.

| kanal | eklem | servo | pin |
|---|---|---|---|
| 1 | Bel | MG996R | D5 |
| 2 | Omuz | MG996R | D6 |
| 3 | Dirsek | MG996R | D7 |
| 4 | Bilek dönüş | SG90 | D8 |
| 5 | Bilek eğim | SG90 | D9 |
| 6 | Kıskaç | SG90 | D10 |

### 2. Linux'ta seri port izni (bir kere)
Kullanıcı `dialout` grubunda değilse port açılamaz (pencere bunu söyler):

```bash
sudo usermod -aG dialout $USER
```

ardından oturumu kapatıp açın.

### 3. Uygulamada bağlanın
Menü **Arduino > Bağlantı penceresi** (ya da alt durum çubuğundaki "Arduino: ..." yazısına tıklayın):
- **Port** listesinde Arduino'lar (`/dev/ttyACM*`, CH340 klonlar `/dev/ttyUSB*`) en üstte görünür; **Yenile** listeyi
  günceller, listede olmayan bir port **Port yolu** kutusuna elle yazılabilir. Baud: 115200.
- **Bağlan**: Uno port açılınca kendini yeniden başlatır; uygulama kartın `READY` mesajını bekler, firmware'i doğrular
  (`INFO`) ve bağlanır. Durum renkli gösterilir (gri: bağlı değil, sarı: bağlanıyor, yeşil: bağlı, kırmızı: hata ve
  nedeni). Bağlıyken gecikme (PING), son yanıt zamanı, gönderilen/onaylanan hareket ve hata sayısı görünür.
- Bağlantı sürekli izlenir: kablo çekilirse ya da kart 2 s yanıt vermezse "Bağlantı koptu" hatası verilir ve canlı
  takip kapanır. Servolar çalışmaya başlayınca kart yeniden başlarsa (besleme yetersiz) bu da bildirilir.
- **Robotu takip et (canlı)**: gerçek kol, ekrandaki modelin eklem açılarını izler — eklem kaydırıcıları, IK,
  *Çalıştır* animasyonu doğrudan servolara gider (en fazla 50 güncelleme/s). **Hız sınırı** (°/s) kartta uygulanır,
  böylece takip açıldığında kol modelin pozuna yavaşça gider.
- **ACİL DURDUR**: kartın hareket kuyruğunu boşaltır, kol olduğu yerde durur, takip ve animasyon kapanır.
  **Başlangıç pozu** firmware'deki başlangıç pozuna (90/150/35/140/85/80) gider; **Servoları serbest bırak** servolara
  darbe göndermeyi keser.
- **Haberleşme günlüğü**: gönderilen (`>`) ve gelen (`<`) satırlar; alttaki kutudan elle komut gönderilebilir.

### 4. Kalibrasyon
Modeldeki eklem açısı ile servo açısı arasındaki ilişki: **servo = merkez + yön × eklem açısı** (min..max ile sınırlı).
Her kanal için:
1. Takip kapalıyken modelde eklemi 0°'ye getirin (Robot paneli).
2. **Elle** kaydırıcısıyla gerçek servoyu modeldeki duruşa getirin, **Merkez yap**'a basın.
3. Eklemi modelde biraz çevirin, takibi açın: gerçek kol ters dönüyorsa **Yön**'ü −1 yapın.
4. **Min / Max** ile servonun mekanik sınırlarını girin (kart da bu sınırları uygular).
5. **Ayarları kaydet**: robot klasörüne `arduino.pegasus` yazılır, robot yüklenince otomatik okunur.

Varsayılan eşleme: kanal 1..6 = modelin bağımsız eklemleri tabandan uca (Bel, Omuz, Dirsek, Bilek dönüş, Bilek eğim,
Kıskaç), merkez 90°, yön +1.

### Haberleşme protokolü
115200 baud, satır tabanlı metin (`\n`). Açılar servo derecesinin 10 katı (0..1800 → 0.1° çözünürlük).
Arduino IDE seri monitörü ile de denenebilir.

| komut | cevap | anlamı |
|---|---|---|
| `PING` | `PONG` | canlılık kontrolü (uygulama 0.5 s'de bir gönderir) |
| `INFO` | `INFO pegasus-arm 1.0 ch=6 pins=5,6,7,8,9,10` | firmware kimliği |
| `M <seq> <ms> <a1>..<a6>` | `A <seq> <boş yer>` | 6 servoya birlikte, `<ms>` içinde doğrusal git (8 adımlık kuyruğa) |
| `S <kanal> <a>` | `OK` | tek servo |
| `V <°/s>` | `OK` | hız sınırı |
| `L <kanal> <min> <max>` | `OK` | kanal sınırları |
| `HOME`, `STOP`, `ATTACH`, `DETACH` | `OK` | başlangıç pozu, dur, servoları sür / bırak |
| `GET` | `POS <hareket> <a1>..<a6> <boş yer>` | anlık konumlar |
| (açılış) | `READY pegasus-arm 1.0` | kart hazır |
| hatalı komut | `ERR <kod> <açıklama>` | 1 bilinmeyen, 2 parametre, 3 kuyruk dolu, 4 satır uzun, 5 zaman aşımı |

Kart her `M` adımını kendi içinde 100 Hz ile ara değerler ve adımları boşluksuz zincirler; süre, hız sınırını
aşmayacak şekilde uzatılır. Uygulama kuyruktaki boş yer bilgisine göre gönderir, bu yüzden gecikme birikmez (her
zaman en güncel hedef gider). Güvenlik: `M` akışı sırasında bilgisayardan 1.5 s komut gelmezse kart kendiliğinden durur.

### Kart olmadan denemek: simülatör
`build/pegasus_arduino_sim`, **aynı firmware kodunu** (`pegasus_arm.ino`) bilgisayarda derleyip sahte bir seri port
üzerinden çalıştırır:

```bash
./build/pegasus_arduino_sim
```

Uygulamada port listesinde "Pegasus Arduino simülatörü" (`/tmp/pegasus-arduino-sim`) görünür. Gerçek Uno gibi port
açılınca yeniden başlar. Otomatik test (simülatörü kendisi başlatır; bağlanma, canlı takip, akış, acil durdurma, elle
sürme, kalibrasyon kaydı, kablo çekilmesi, olmayan port ve izin hatası):

```bash
./build/pegasus --selftest-arduino "robot kol stl parçaları" /tmp/pegasus-arduino-test
```

Aynı test gerçek kartla da çalışır (servolar bağlıysa kol hareket eder!):

```bash
PEGASUS_TEST_PORT=/dev/ttyACM0 ./build/pegasus --selftest-arduino "robot kol stl parçaları" /tmp/pegasus-arduino-hw
```

## Kod yapısı

```
src/
  render/   VulkanContext (cihaz, swapchain, MSAA), Renderer (pipeline'lar, GPU mesh önbelleği), Png
  scene/    Mesh (kaynaklama, kenar/yüzey topolojisi, BVH), StlLoader, Primitives, Scene (nesne + dönüşüm)
  geom/     Intersect (ışın/üçgen, üçgen/üçgen SAT), Collision (BVH-BVH çarpışma sistemi)
  robot/    Robot (parça ağacı, eklemler, ileri kinematik, kayıt), JointDetect (otomatik eklem algılama, eksen uydurma),
            IkSolver (Blender tarzı SDLS/DLS ters kinematik)
  motion/   Expr (formül derleyici), MotionLibrary (.pmotion/.pscan/.ptool), Seam (kenarlardan yol çerçevesi),
            SurfaceScan (yüzey tarama yolu), PathPlanner (örnekleme, IK ile yol takibi, zamanlı yörünge)
library/    motions/*.pmotion (kenar hareketleri), scans/*.pscan (tarama desenleri), tools/*.ptool (başlıklar)
  hw/       SerialPort (Linux seri port), ArduinoLink (arka plan iş parçacığında protokol, akış, bağlantı izleme)
  editor/   Editor (arayüz, araçlar, gizmo, sürükleme), Editor_Robot (robot paneli), Editor_Motion (hareket paneli),
            Editor_Arduino (Arduino penceresi, servo kalibrasyonu),
            Editor_Select (bağlı / döngü / fırça seçimi, büyüt-küçült),
            Picking, Camera, FileBrowser, SelfTest
arduino/    pegasus_arm/pegasus_arm.ino (Uno firmware'i), sim/ (firmware'i PC'de çalıştıran simülatör)
shaders/    mesh.vert/frag (türev tabanlı düz gölgeleme), line.vert/frag
```

Otomatik arayüz testi (gerçek fare olaylarını simüle eder, her adımın ekran görüntüsünü kaydeder):

```bash
./build/pegasus --selftest /tmp/pegasus-test
./build/pegasus --selftest-robot "robot kol stl parçaları" /tmp/pegasus-robot-test
./build/pegasus --selftest-motion "robot kol stl parçaları" /tmp/pegasus-motion-test
./build/pegasus --selftest-scan "robot kol stl parçaları" /tmp/pegasus-scan-test
./build/pegasus --selftest-select "robot kol stl parçaları" /tmp/pegasus-select-test
./build/pegasus --selftest-arduino "robot kol stl parçaları" /tmp/pegasus-arduino-test
```

## Notlar
- Bağlantı şeması `Arduino-Robot-Arm-Schematic-Circuit-Diagram.png` (HC-05 hariç). Telefon / Bluetooth kontrolü yoktur;
  kol yalnızca bu uygulamadan, USB seri üzerinden kontrol edilir.
