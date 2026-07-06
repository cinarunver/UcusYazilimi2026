import serial
import serial.tools.list_ports
import struct
import time
import threading
import sys

# --- SABİT AYARLAR ---
# Ek-7 Tablo 7: Baud 115200. (ESP32 firmware BAUD_TTL ile birebir aynı olmalı.)
BAUD_RATE = 115200
HEADER_CMD = 0xAA    # Komut (PC->UKB) ve Durum paketi (UKB->PC) header'ı
HEADER_DATA = 0xAB   # SİT telemetri ve SUT veri header'ı
FOOTER1 = 0x0D
FOOTER2 = 0x0A
# Checksum = checksum'dan ONCEKI tum byte'larin toplami (mod 256) — EK-7 Bolum 3.
# Komut icin: Header + Command (logic analyzer ile dogrulandi: 0xAA+0x20=0xCA).
# EK-7 Tablo 1'deki 0x8C/0x8E/0x90 HATALI, kullanma.

CMD_SIT_BASLAT = 0x20
CMD_SUT_BASLAT = 0x22
CMD_DURDUR = 0x24

# SUT Durum Bilgilendirme bitleri (Tablo 5) — UKB'den gelen 2 byte'ın açılımı
DURUM_BITLERI = [
    "Kalkis algilandi",             # Bit 0
    "Motor yanma suresi doldu",     # Bit 1
    "Min irtifa esigi asildi",      # Bit 2
    "Aci/ivme esigi asildi",        # Bit 3
    "Alcalma basladi",              # Bit 4
    "Suruklenme (drogue) emri",     # Bit 5
    "Belirlenen irtifanin altinda", # Bit 6
    "Ana parasut emri",             # Bit 7
]

def decode_durum(bits):
    """16 bitlik durum degerini aktif asama isimlerine cevirir."""
    aktif = [ad for i, ad in enumerate(DURUM_BITLERI) if bits & (1 << i)]
    return f"0x{bits:04X} -> " + (", ".join(aktif) if aktif else "(hicbiri)")

def select_serial_port():
    """Sistemdeki aktif seri portları listeler ve kullanıcıya seçtirir."""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("\n[!] HATA: Hiçbir seri port bulunamadı! ESP32'nin bağlı olduğundan emin olun.")
        sys.exit()

    print("\n--- MEVCUT SERİ PORTLAR ---")
    for i, port in enumerate(ports):
        print(f"{i}: {port.device} - {port.description}")
    
    while True:
        try:
            choice = input("\nPort Numarası Seç (Çıkış için Enter): ")
            if choice == "": # Düz enter basıldıysa kapat
                sys.exit()
            
            index = int(choice)
            if 0 <= index < len(ports):
                return ports[index].device
            else:
                print(f"Hata: 0 ile {len(ports)-1} arasında bir numara girin.")
        except ValueError:
            print("Hata: Lütfen geçerli bir sayı girin.")

class RocketTester:
    def __init__(self, port, baud):
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            print(f"\n[+] Bağlantı Başarılı: {port} @ {baud}")
        except Exception as e:
            print(f"\n[!] Bağlantı Hatası: {e}")
            sys.exit()
        
        self.running = True
        self.raw = False   # True iken gelen HER byte hex+ascii olarak basilir (teshis modu)
        self.read_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.read_thread.start()

    def send_command(self, cmd_byte):
        chk = (HEADER_CMD + cmd_byte) & 0xFF   # Header + Command
        packet = struct.pack("BBBBB", HEADER_CMD, cmd_byte, chk, FOOTER1, FOOTER2)
        self.ser.write(packet)
        print(f"\n>> Komut Gönderildi: {hex(cmd_byte)} (chk={hex(chk)})")

    def send_sut_data(self, alt, press, ax, ay, az, roll, pitch, yaw):
        payload = struct.pack("<ffffffff", alt, press, ax, ay, az, roll, pitch, yaw)
        chk = (HEADER_DATA + sum(payload)) & 0xFF   # Header (0xAB) + payload
        packet = struct.pack("B", HEADER_DATA) + payload + struct.pack("BBB", chk, FOOTER1, FOOTER2)
        self.ser.write(packet)

    def run_sut_scenario(self):
        """Tam sentetik ucus profili: kalkis -> apogee -> drogue -> ana parasut -> inis.
        Deger esikleri UKB firmware'i ile uyumlu secilmistir:
          - Kalkis:  ivmeZ > 20 m/s^2  (KALKIS_IVME_ESIGI)
          - Apogee:  max_irtifa - irtifa > 15 m ve dikey hiz < 0 ve egim < 10 deg
          - Ana par: irtifa < 550 m ve max_irtifa > 550 m  (AYRILMA2_MESAFE)
          - Inis:    dikey hiz ~0 ve irtifa < 20 m
        """
        P0 = 1013.25
        dt = 0.05          # 20 Hz gonderim (< 1 sn Test Timeout)
        TAVAN = 2000.0     # apogee irtifasi (m)

        def basinc_hesap(h):
            # Basit barometrik model (UKB SUT'ta basinci karar icin kullanmaz, gorsel amacli)
            return P0 * pow(max(0.0, 1.0 - 2.25577e-5 * h), 5.25588)

        print("\n>> SENARYO: SUT baslatiliyor, UKB 1 sn sonra aktif olacak...")
        self.send_command(CMD_SUT_BASLAT)
        time.sleep(1.2)    # Uygulama Plani (c): komut onayindan 1 sn sonra SUT aktif

        # --- Faz 1: Rampa / Kalkis tetigi (ivmeZ > 20) ---
        print(">> Faz 1: Kalkis (ivmeZ=30 m/s^2)")
        for _ in range(10):   # ~0.5 sn yuksek ivme
            self.send_sut_data(0.0, basinc_hesap(0.0), 0.0, 0.0, 30.0, 0.0, 0.0, 0.0)
            time.sleep(dt)

        # --- Faz 2: Yukselis 0 -> TAVAN ---
        print(">> Faz 2: Yukselis 0 -> %.0f m" % TAVAN)
        h = 0.0
        while h < TAVAN:
            h = min(TAVAN, h + 40.0)
            az = max(0.0, 15.0 - h / 200.0)      # motor sonrasi ivme azaliyor
            self.send_sut_data(h, basinc_hesap(h), 0.0, 0.0, az, 0.0, 0.0, 0.0)
            time.sleep(dt)

        # --- Faz 3: Inis TAVAN -> 0 (apogee, drogue, ana parasut burada tetiklenir) ---
        print(">> Faz 3: Inis %.0f -> 0 m (kurtarma sinyalleri bekleniyor)" % TAVAN)
        while h > 0.0:
            h = max(0.0, h - 25.0)
            self.send_sut_data(h, basinc_hesap(h), 0.0, 0.0, -3.0, 0.0, 0.0, 0.0)
            time.sleep(dt)

        # --- Faz 4: Yerde (inis tespiti icin sabit + hiz ~0) ---
        print(">> Faz 4: Yer temasi")
        for _ in range(15):
            self.send_sut_data(0.0, basinc_hesap(0.0), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            time.sleep(dt)

        print("\n>> Senaryo tamamlandi. Durum bitleri yukarida izlendi. Bitirmek icin '3' (Durdur).")

    def raw_dinle(self, saniye, komut=None):
        """Komut gonderir (opsiyonel) ve <saniye> boyunca gelen HER byte'i ham gosterir.
        UKB'den bir sey gelip gelmedigini (paket bozuk olsa bile) gormek icin."""
        if komut is not None:
            self.send_command(komut)
        print(f">> {saniye} sn ham dinleme basladi... (gelen her byte hex+ascii)")
        self.raw = True
        time.sleep(saniye)
        self.raw = False
        print(">> Ham dinleme bitti.")

    def receive_loop(self):
        # Byte akışını header'a göre ayrıştırır:
        #   0xAB + 35 byte -> SİT telemetri paketi (Tablo 3, 36 byte)
        #   0xAA + 5  byte -> SUT durum bilgilendirme paketi (Tablo 6, 6 byte)
        # RAW modunda (self.raw) hicbir sey ayristirilmaz, gelen her byte dokulur.
        while self.running:
            try:
                if self.raw:
                    d = self.ser.read(64)
                    if d:
                        hx = d.hex(" ").upper()
                        asc = "".join(chr(x) if 32 <= x < 127 else "." for x in d)
                        print(f"[RAW] {hx}   | {asc}")
                    continue

                b = self.ser.read(1)
                if not b:
                    continue

                if b == b'\xab':
                    data = self.ser.read(35)
                    if len(data) == 35 and data[-2:] == b'\r\n':
                        vals = struct.unpack("<ffffffff", data[0:32])
                        print(f"\r[SİT] Irtifa:{vals[0]:.2f}m Basinc:{vals[1]:.2f}hPa "
                              f"ivX:{vals[2]:.2f} ivY:{vals[3]:.2f} ivZ:{vals[4]:.2f} "
                              f"aciX:{vals[5]:.2f} aciY:{vals[6]:.2f} aciZ:{vals[7]:.2f}      ", end="")

                elif b == b'\xaa':
                    rest = self.ser.read(5)  # Data1, Data2, CHK, 0x0D, 0x0A
                    if len(rest) == 5 and rest[-2:] == b'\r\n':
                        data1, data2 = rest[0], rest[1]
                        bits = data1 | (data2 << 8)
                        print(f"\n[DURUM] {decode_durum(bits)}")
            except Exception:
                break
            time.sleep(0.005)

def main():
    print("================================================")
    print("   TRAKYA ROKET 2026 - SİT/SUT TEST PANELİ")
    print("================================================")
    
    selected_port = select_serial_port()
    tester = RocketTester(selected_port, BAUD_RATE)
    
    print("\n--- KONTROL MENÜSÜ ---")
    print("1: SIT Başlat (Sensör Verilerini İzle)")
    print("2: SUT Başlat (Yapay Veri Girişi Hazırla)")
    print("3: Durdur (Testi Bitir)")
    print("4: SUT Senaryosu (Uçuş Simülasyonu - kalkış→apogee→iniş)")
    print("5: TEŞHİS — SIT gönder + 10 sn HAM hex dinle (gelen ne varsa yaz)")
    print("6: TEŞHİS — komutsuz 10 sn HAM hex dinle")
    print("Q: Çıkış (veya Enter)")

    try:
        while True:
            choice = input("\nSeçiminiz: ").upper()
            
            # Düz enter veya 'Q' basıldıysa çıkış yap
            if choice == "" or choice == "Q":
                break
                
            if choice == '1':
                tester.send_command(CMD_SIT_BASLAT)
            elif choice == '2':
                tester.send_command(CMD_SUT_BASLAT)
            elif choice == '3':
                tester.send_command(CMD_DURDUR)
            elif choice == '4':
                tester.run_sut_scenario()
            elif choice == '5':
                tester.raw_dinle(10, komut=CMD_SIT_BASLAT)
            elif choice == '6':
                tester.raw_dinle(10)
    except KeyboardInterrupt:
        pass
    finally:
        tester.running = False
        print("\nBağlantı kapatıldı, uygulama sonlandırıldı.")

if __name__ == "__main__":
    main()
