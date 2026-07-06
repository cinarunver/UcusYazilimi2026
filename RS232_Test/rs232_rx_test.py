import serial
import serial.tools.list_ports
import threading
import sys

# ─────────────────────────────────────────────────────────────
#  RS232 RX TEST MONITORU  (rs232_echo.cpp firmware'i ile calisir)
#  - Ekranda "HB: N" satirlari akmali  -> TX/firmware canli
#  - Enter'a basinca SIT komutu (AA 20 8C 0D 0A) gonderilir
#      -> ">> RX: 0xAA ..." cikarsa RX SAGLAM
#      -> hic cikmazsa RX KOPUK (donanim)
# ─────────────────────────────────────────────────────────────

BAUD = 115200
# SIT Baslat: Header 0xAA + Command 0x20 + Checksum (0xAA+0x20=0xCA) + 0x0D + 0x0A
SIT_KOMUT = bytes([0xAA, 0x20, 0xCA, 0x0D, 0x0A])


def port_sec():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("[!] Hicbir seri port yok. RS232 adaptorunu tak.")
        sys.exit()
    print("\n--- SERI PORTLAR ---")
    for i, p in enumerate(ports):
        print(f"{i}: {p.device} - {p.description}")
    while True:
        s = input("\nPort no: ")
        try:
            i = int(s)
            if 0 <= i < len(ports):
                return ports[i].device
        except ValueError:
            pass
        print("Gecersiz.")


def main():
    port = port_sec()
    ser = serial.Serial(port, BAUD, timeout=0.2)
    print(f"\n[+] Baglandi: {port} @ {BAUD}")
    print("Dinleniyor... 'HB:' satirlari akmali.")
    print("Komut gondermek icin ENTER'a bas. Cikis icin 'q' + ENTER.\n")

    def oku():
        while True:
            d = ser.read(256)
            if d:
                sys.stdout.write(d.decode(errors="replace"))
                sys.stdout.flush()

    threading.Thread(target=oku, daemon=True).start()

    while True:
        cmd = input()
        if cmd.strip().lower() == "q":
            break
        ser.write(SIT_KOMUT)
        print("\n>>> SIT komutu gonderildi: AA 20 8C 0D 0A  (RX calisiyorsa >> RX: satirlari gelmeli)\n")

    ser.close()
    print("Kapatildi.")


if __name__ == "__main__":
    main()
