#!/usr/bin/env python3
# ============================================================================
# lidar_probe.py  —  JETSON'da çalışır (ROS GEREKMEZ, yalnız pyserial)
#
# YDLIDAR T-mini Plus'ın GERÇEK paket formatını ve checksum formülünü tespit
# eder. Sürücüde "Bozuk paket" seli varsa bu araçla neyin uyuşmadığı bulunur:
#
#   1. Ham SCAN akışından birkaç saniye veri yakalar.
#   2. 0xAA 0x55 başlıkları arası mesafeden örnek boyutunu (2/3 bayt) ÖLÇER.
#   3. Her paket için 4 farklı checksum hipotezini dener, tutma oranını yazar:
#        A) 2B örnek: XOR(PH, CT|LSN, FSA, LSA, mesafe-kelimeleri)   [sürücüdeki]
#        B) 3B örnek: mesafe kelimeleri (intensity XOR'a katılmaz)   [sürücüdeki]
#        C) 3B örnek: intensity baytı AYRI kelime olarak XOR'a katılır [SDK tarzı]
#        D) Tüm paket baytları sırayla 16-bit çiftlenir (CS alanı hariç)
#   4. Örnek mesafeleri iki ölçek hipoteziyle yazar (ham/4 mm ve ham mm).
#
# Çalıştırma (önce sürücüyü durdur: pkill -f ydlidar_node):
#   python3 lidar_probe.py [/dev/ttyUSB0] [süre_sn]
# ============================================================================

import struct
import sys
import time
from collections import Counter

try:
    import serial
except ImportError:
    raise SystemExit("pyserial gerekli: pip3 install pyserial")

CMD_STOP = b'\xA5\x65'
CMD_SCAN = b'\xA5\x60'
CMD_INFO = b'\xA5\x90'


def read_for(ser, seconds):
    end = time.monotonic() + seconds
    buf = bytearray()
    while time.monotonic() < end:
        chunk = ser.read(4096)
        if chunk:
            buf.extend(chunk)
    return bytes(buf)


def device_info(ser):
    """GET_DEVICE_INFO dener; model/firmware baytlarını ham yazar (best-effort)."""
    ser.reset_input_buffer()
    ser.write(CMD_INFO)
    time.sleep(0.3)
    raw = ser.read(64)
    if len(raw) >= 7 and raw[0] == 0xA5 and raw[1] == 0x5A:
        payload = raw[7:]
        print(f"[info] DEVICE_INFO ham: {payload.hex(' ')}")
        if len(payload) >= 3:
            print(f"[info] model={payload[0]} firmware={payload[2]}.{payload[1]}")
    else:
        print(f"[info] DEVICE_INFO cevabı çözülemedi ({len(raw)} bayt): {raw.hex(' ')}")


def find_headers(data):
    """0xAA 0x55 başlık konumları."""
    pos, i = [], 0
    while True:
        i = data.find(b'\xAA\x55', i)
        if i < 0:
            break
        pos.append(i)
        i += 2
    return pos


def xor_words(words):
    c = 0
    for w in words:
        c ^= w
    return c & 0xFFFF


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0

    print(f"Port açılıyor: {port} @ 230400 ...")
    ser = serial.Serial(port, 230400, timeout=0.05)

    ser.write(CMD_STOP)
    time.sleep(0.2)
    ser.reset_input_buffer()

    device_info(ser)

    print(f"SCAN başlatılıyor, {dur:.0f} sn ham veri toplanıyor ...")
    ser.write(CMD_SCAN)
    data = read_for(ser, dur)
    ser.write(CMD_STOP)
    time.sleep(0.1)
    ser.close()
    print(f"Toplandı: {len(data)} bayt")
    if len(data) < 1000:
        raise SystemExit("Çok az veri — lidar dönüyor mu, port doğru mu?")

    heads = find_headers(data)
    print(f"0xAA55 başlık sayısı: {len(heads)}")
    if len(heads) < 10:
        raise SystemExit("Yeterli başlık yok — SCAN akışı gelmiyor olabilir.")

    # --- 1) Örnek boyutunu başlıklar arası mesafeden ÖLÇ ---------------------
    sb_votes = Counter()
    for a, b in zip(heads, heads[1:]):
        if a + 10 > len(data):
            continue
        lsn = data[a + 3]
        gap = b - a
        if lsn > 0 and gap > 10:
            per = (gap - 10) / lsn
            if abs(per - round(per)) < 1e-9 and 1 <= round(per) <= 4:
                sb_votes[int(round(per))] += 1
    print(f"örnek-boyutu oyları (başlık aralığından): {dict(sb_votes)}")
    sb = sb_votes.most_common(1)[0][0] if sb_votes else 2
    print(f"==> ölçülen örnek boyutu: {sb} bayt/örnek")

    # --- 2) Checksum hipotezleri --------------------------------------------
    tried = Counter()
    match = Counter()
    dists_q6 = []   # ham/4 hipotezi (mm)
    dists_raw = []  # ham=mm hipotezi
    lsn_votes = Counter()

    for a in heads:
        if a + 10 > len(data):
            continue
        ct, lsn = data[a + 2], data[a + 3]
        fsa = data[a + 4] | (data[a + 5] << 8)
        lsa = data[a + 6] | (data[a + 7] << 8)
        cs = data[a + 8] | (data[a + 9] << 8)
        plen = 10 + lsn * sb
        if lsn == 0 or a + plen > len(data):
            continue
        smp = data[a + 10:a + plen]
        lsn_votes[lsn] += 1

        base = [0x55AA, ct | (lsn << 8), fsa, lsa]

        if sb == 2:
            words = [smp[i] | (smp[i + 1] << 8) for i in range(0, len(smp), 2)]
            tried['A: 2B mesafe-kelime XOR'] += 1
            if xor_words(base + words) == cs:
                match['A: 2B mesafe-kelime XOR'] += 1
            for w in words[:2]:
                dists_q6.append(w * 0.25)
                dists_raw.append(float(w))
        else:  # sb == 3
            iw = [smp[i] for i in range(0, len(smp), 3)]                       # intensity
            dw = [smp[i + 1] | (smp[i + 2] << 8) for i in range(0, len(smp), 3)]  # mesafe
            tried['B: 3B yalnız-mesafe XOR'] += 1
            if xor_words(base + dw) == cs:
                match['B: 3B yalnız-mesafe XOR'] += 1
            tried['C: 3B intensity+mesafe XOR'] += 1
            if xor_words(base + iw + dw) == cs:
                match['C: 3B intensity+mesafe XOR'] += 1
            for w in dw[:2]:
                dists_q6.append(w * 0.25)
                dists_raw.append(float(w))
            # T-mini varyantı: mesafe üst 14 bit olabilir (alt 2 bit bayrak)
            # bilgi amaçlı ölçek karşılaştırmasında görünür.

        # D: tüm paket baytları sırayla 16-bit çiftlenir, CS alanı 0 sayılır
        pkt = bytearray(data[a:a + plen])
        pkt[8] = pkt[9] = 0
        words = [pkt[i] | (pkt[i + 1] << 8) if i + 1 < len(pkt) else pkt[i]
                 for i in range(0, len(pkt), 2)]
        tried['D: bayt-sıralı çift XOR'] += 1
        if xor_words(words) == cs:
            match['D: bayt-sıralı çift XOR'] += 1

    print(f"\npaket başına örnek sayısı (LSN) dağılımı: {dict(lsn_votes)}")
    print("\n--- CHECKSUM HİPOTEZ SONUÇLARI ---")
    for name in sorted(tried):
        t, m = tried[name], match[name]
        pct = 100.0 * m / t if t else 0.0
        mark = "  <== EŞLEŞME!" if pct > 90 else ""
        print(f"  {name:32s}: {m:5d}/{t:5d}  (%{pct:.1f}){mark}")

    # --- 3) Mesafe ölçeği karşılaştırması -----------------------------------
    def stats(v):
        v = [x for x in v if x > 0]
        if not v:
            return "veri yok"
        v.sort()
        return (f"min {v[0]:.0f}  medyan {v[len(v)//2]:.0f}  "
                f"maks {v[-1]:.0f} mm")

    print("\n--- MESAFE ÖLÇEĞİ (ilk 2 örnek/paket) ---")
    print(f"  ham/4 hipotezi : {stats(dists_q6)}")
    print(f"  ham=mm hipotezi: {stats(dists_raw)}")
    print("  (odadaki gerçek duvar mesafenle karşılaştır; mantıklı olan doğru ölçektir)")

    print("\nBitti. Bu çıktının tamamını paylaş.")


if __name__ == '__main__':
    main()
