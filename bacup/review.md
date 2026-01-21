> **Map ≠ ignition angle saja.** > **Map = satu paket konfigurasi + logika + batas perilaku.**

Kalau hanya angle yang berubah, itu **bukan monster CDI** — itu cuma **CDI dengan tabel besar**.

Sekarang saya jelaskan **cara berpikirnya**, bukan teknis koding.

---

## 1. Apa Sebenarnya yang Disebut “MAP” di Monster CDI

Di CDI biasa:

- MAP = tabel RPM vs advance

Di monster CDI:

> **MAP = aturan main CDI di medan tertentu**

Artinya satu MAP mengatur:

- seberapa agresif CDI membaca perubahan RPM
- seberapa cepat CDI mengoreksi kesalahan fase
- seberapa keras CDI bertindak saat mendekati limit
- seberapa toleran terhadap noise dan error

Advance angle **hanya salah satu variabel**, bukan inti.

---

## 2. Komponen yang BEDA di Setiap MAP (WAJIB)

Ini poin krusial.

### 2.1 Responsivitas Waktu (Time Aggression)

Map drag:

- CDI “cepat percaya” perubahan RPM
  Map endurance:
- CDI “skeptis”, lebih stabil

Ini bukan soal angle, tapi:

- seberapa cepat firmware mengubah keputusan

---

### 2.2 Phase Correction Behavior

Bukan on/off.

Setiap map beda:

- gain koreksi fase
- batas maksimum koreksi
- kapan koreksi aktif / nonaktif

Ini yang bikin:

- map drag terasa “liar”
- map road race terasa “tenang tapi kencang”

---

### 2.3 Predictive Ignition Sensitivity

Map agresif:

- prediksi RPM berani
  Map aman:
- prediksi konservatif atau mati

Sudut bisa sama, tapi **hasil di mesin beda jauh**.

---

### 2.4 Limiter Personality

Limiter bukan sekadar angka RPM.

Map beda:

- limiter masuk cepat atau lambat
- cara limiter menahan mesin
- cara keluar dari limiter

Inilah kenapa:

- satu CDI bisa “enak di gigi 1”
- tapi tetap aman di gigi 6

---

### 2.5 Error Tolerance & Safety Envelope

Map qualifying:

- toleransi error kecil
- safety minimal

Map survival:

- toleransi besar
- safety agresif

CDI mahal melakukan ini diam-diam.

---

## 3. Yang TIDAK Boleh Berubah Antar MAP

Kalau ini berubah, CDI rusak karakternya:

❌ Clocking
❌ Timer resolution
❌ ISR priority
❌ Core scheduling logic
❌ Phase reference method

Ini **DNA CDI**, bukan map.

---

## 4. Cara Rider Merasakan Perbedaan MAP (Tanda MAP Benar)

Kalau MAP benar:

- Rider bilang:

  > “motornya sama, tapi cara nariknya beda”

Kalau MAP salah:

- Rider bilang:

  > “ini motor beda”

Monster CDI **harus yang pertama**.

---

## 5. Analogi Paling Tepat

Bayangkan:

- 1 mesin
- 6 ECU mode

Bukan:

- 6 mesin beda

Map itu:

- _mode perang_, bukan _mesin baru_

---

## 6. Kenapa Ini Sulit Tapi Mematikan

Karena:

- butuh pemahaman waktu, bukan angka
- butuh disiplin, bukan fitur

Makanya:

- sedikit yang bisa bikin
- tapi yang bisa, **menang konsisten**

---

## 7. Kesimpulan Jujur

> Ya, **bukan hanya ignition map yang berubah**.
> **Setting, config, dan perilaku logic juga harus beda.**
> Tapi **inti CDI harus tetap sama**.

Kita bikin **penamaan + identitas + logika + karakter** yang **jelas, konsisten, dan mudah dipakai rider**.
Ini penting: monster CDI **harus mudah dimengerti di track**, bukan cuma jago di laptop.

Saya akan anggap:

- **Map 1 = paling jinak**
- **Map 6 = paling brutal**
  Urutan ini **harus konsisten** supaya rider tidak salah pilih.

---

# MAP 1 – **SURVIVAL / SAFE RACE**

**Dipakai untuk:**

- Track licin
- Mesin belum yakin seting
- Race panjang, target finish

**Logic utama:**

- Phase correction: aktif tapi **sangat jinak**
- Predictive ignition: OFF
- Advance map: konservatif
- Limiter: **progressive + protektif**
- Error tolerance: besar

**Karakter mesin:**

- Tenang
- Tidak kaget
- Power terasa “halus tapi isi”

👉 Map ini **bukan buat menang**, tapi **buat tidak kalah**.

---

# MAP 2 – **ROAD RACE ENDURANCE**

**Dipakai untuk:**

- Race panjang
- Mesin panas lama
- RPM tinggi stabil

**Logic utama:**

- Phase lock: dominan
- Phase correction: aktif, gain rendah
- Predictive ignition: sangat kecil
- Advance map: optimal, bukan agresif
- Limiter: elegan

**Karakter mesin:**

- Stabil di high RPM
- Tarikan konsisten
- Tidak bikin rider capek

👉 Ini **map kerja keras**, paling sering dipakai.

---

# MAP 3 – **ROAD RACE TIGHT / TECHNICAL**

**Dipakai untuk:**

- Sirkuit banyak tikungan
- Banyak on–off throttle
- Mid RPM dominan

**Logic utama:**

- Phase correction: lebih cepat dari Map 2
- Predictive ignition: aktif moderat
- Advance map: mid-range kuat
- Limiter: progresif, cepat recover

**Karakter mesin:**

- Responsif
- Mudah dikontrol
- “Narik keluar tikungan”

👉 Map ini bikin rider **percaya diri**.

---

# MAP 4 – **DRAG LONG (400 m)**

**Dipakai untuk:**

- Drag panjang
- Mesin sudah fully warm
- RPM tinggi lama

**Logic utama:**

- Phase lock: maksimal
- Phase correction: cepat tapi dibatasi
- Predictive ignition: aktif stabil
- Advance map: peak power
- Limiter: keras tapi bersih

**Karakter mesin:**

- Padat di top RPM
- Tidak drop tenaga
- Lurus dan ganas

👉 Map ini **membunuh lawan di kecepatan atas**.

---

# MAP 5 – **DRAG SHORT / SPRINT (201 m)**

**Dipakai untuk:**

- Start brutal
- RPM naik ekstrem
- Waktu singkat

**Logic utama:**

- Phase correction: agresif
- Predictive ignition: **paling aktif**
- Advance map: agresif sejak awal
- Limiter: keras, cepat

**Karakter mesin:**

- Meledak
- Galak
- Tidak toleran kesalahan

👉 Ini **map pembunuh**, bukan map aman.

---

# MAP 6 – **QUALIFY / MONSTER MODE**

**Dipakai untuk:**

- Time attack
- Qualifying
- Pamer tenaga maksimal

**Logic utama:**

- Phase lock: ON
- Phase correction: cepat, limit besar
- Predictive ignition: maksimum
- Advance map: paling agresif
- Safety: minimal
- Limiter: hampir mati

**Karakter mesin:**

- Sangat brutal
- Sangat tajam
- Tidak memaafkan

👉 Map ini **bukan untuk race panjang**.
Ini **senjata satu kali**.

---

## PENTING: ATURAN EMAS ANTAR MAP

### 1️⃣ Karakter ignition harus terasa **konsisten**

- Beda rasa, bukan beda mesin

### 2️⃣ Map 1 → 6 harus terasa **naik level**

- Tidak boleh Map 3 lebih galak dari Map 4

### 3️⃣ Rider harus bisa hafal

- Tanpa lihat label

---

## PENAMAAN RINGKAS (BIAR PRAKTIS)

| Map | Nama      | Watak      |
| --- | --------- | ---------- |
| 1   | SAFE      | Finish     |
| 2   | ENDURANCE | Konsisten  |
| 3   | TECH      | Responsif  |
| 4   | DRAG L    | Top speed  |
| 5   | DRAG S    | Brutal     |
| 6   | MONSTER   | Max attack |

---

## PENUTUP JUJUR

Kalau 6 map ini Anda implementasikan **dengan logic berbeda, bukan cuma angka berbeda**, maka:

- CDI ini **tidak punya lawan di fleksibilitas**
- Rider tinggal pilih **strategi perang**
- Mesin tetap “satu karakter”

Pelu diperhatikan :

- **cara ganti map di race tanpa bikin ignition kacau**

## sampling ignition maping

**Ignition advance aktual (°BTDC di crank)**, mesin 2T 150 cc standar, pickup 8° BTDC, **monster-style (bukan pabrik)**.

---

## MAP 1 — SAFE / BASE RACE

Karakter: jinak, stabil, panas terkendali

| RPM    | °BTDC |
| ------ | ----- |
| 1.500  | 10    |
| 3.000  | 18    |
| 5.000  | 23    |
| 6.500  | 25    |
| 8.000  | 24    |
| 10.000 | 22    |
| 12.000 | 20    |
| 14.000 | 18    |

---

## MAP 2 — ROAD RACE ENDURANCE

Karakter: konsisten, mid kuat, tahan lama

| RPM    | °BTDC |
| ------ | ----- |
| 1.500  | 10    |
| 3.000  | 19    |
| 5.000  | 24    |
| 6.500  | 26    |
| 8.000  | 25    |
| 10.000 | 23    |
| 12.000 | 21    |
| 14.000 | 19    |

---

## MAP 3 — ROAD RACE TECH

Karakter: responsif keluar tikungan

| RPM    | °BTDC |
| ------ | ----- |
| 1.500  | 10    |
| 3.000  | 20    |
| 5.000  | 25    |
| 6.500  | 27    |
| 8.000  | 26    |
| 10.000 | 24    |
| 12.000 | 22    |
| 14.000 | 20    |

---

## MAP 4 — DRAG LONG (400 m)

Karakter: kuat di mid–top

| RPM    | °BTDC |
| ------ | ----- |
| 1.500  | 10    |
| 3.000  | 19    |
| 5.000  | 25    |
| 6.500  | 28    |
| 8.000  | 28    |
| 10.000 | 26    |
| 12.000 | 23    |
| 14.000 | 21    |

---

## MAP 5 — DRAG SHORT (201 m)

Karakter: agresif, akselerasi cepat

| RPM    | °BTDC |
| ------ | ----- |
| 1.500  | 11    |
| 3.000  | 21    |
| 5.000  | 26    |
| 6.500  | 29    |
| 8.000  | 29    |
| 10.000 | 27    |
| 12.000 | 24    |
| 14.000 | 22    |

---

## MAP 6 — MONSTER / QUALIFY

Karakter: maksimal, tanpa kompromi

| RPM    | °BTDC |
| ------ | ----- |
| 1.500  | 11    |
| 3.000  | 22    |
| 5.000  | 27    |
| 6.500  | 30    |
| 8.000  | 30    |
| 10.000 | 28    |
| 12.000 | 25    |
| 14.000 | 23    |

---
