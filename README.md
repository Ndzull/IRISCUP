# Ijul IRIS CUP Archive

<p>bismillah aja dulu</p>

<h2>FLOW PROGRAM (kasar)</h2>
<p>HP(IP WEBCAM) --(WIFI/TCP)--> Laptop(main control program) --(WIFI/TCP)--> ESP32 --(selanjutnya urusan electrical)</p>
<h3>Detail</h3>
<h4>HP(IP WEBCAM)</h4>
<p><ul>Ngambil stream(raw) untuk di proses main program</ul>
<ul>Dihubungkan wireless ke laptop lewat WIFI(hostpot lebih aman)</ul>
</p>
<h4>LAPTOP</h4>
<p><ul>Main control robot</ul>
<ul>Aku pake piton</ul>
<ul>Base Station aku edit dikit dari Base Station FP1</ul>
<ul>Tampilan data yang sudah ada sebelumnya:<br>
-Speed<br>
-Jarak tempuh (mungkin aku hapus)<br>
-Steering Angle<br>
-Lane Detector<br>
-Obstacle<br></ul>
<ul>Target data yang ingin aku tambahkan:<br>
-Tombol start untuk menjalankan robotnya<br>
-Tombol stop untuk memberhentikan robot (ulang dari awal)<br>
-Lane Robot Position Detector(robot ada di jalur kanan atau kiri)<br></ul>
</p>
<h4>ESP32</h4>
<p><ul>cuman buat ngehubungin laptop ke ESP32 pakai WIFI</ul></p>

<h2>PROGRAM EXPLAIN: ALL FLOW</h2>
<pre>
Operator (Base Station)
        │  WebSocket
        ▼
  main.py  ─────────────► vision.py
    │                          │
    │                          ▼
    │                    Hasil vision
    │                          │
    ├──────────────► control.py
    │                    │
    │                    ▼
    │               Steering + Speed
    │
    │ UDP
    ▼
 communication.py ─────────► ESP32 (robot)
        ▲                         │
        │   Data sensor (UDP)     │
        └─────────────────────────┘

Data sensor → main.py → Base Station (update realtime)
</pre>

<h4>Partisi file</h4>
<pre>
mainControl/
├── basestation/
│   ├── app.js
│   ├── index.html
│   ├── logo.png
│   └── styles.css
├── communication.py
├── control.py
├── main.py
├── requirements.txt
└── vision.py
</pre>

<h3>Base Station (Web UI)</h3>

<p>
Folder: <code>basestation/</code><br>
Berisi <code>index.html</code>, <code>app.js</code>, dan <code>styles.css</code>.  
UI ini digunakan operator untuk mengontrol robot.
</p>

<ul>
  <li>Mengirim perintah: <em>start</em>, <em>stop</em>, <em>manual/auto</em>, <em>target speed</em>.</li>
  <li>Menerima telemetri dari robot: sudut, speed, status lane, obstacle info.</li>
  <li>Berkomunikasi dengan <code>main.py</code> menggunakan WebSocket.</li>
</ul>

<hr>

<h3>main.py (Program Utama)</h3>

<p>
File inti yang menjalankan seluruh logika sistem.  
Tugas utamanya:
</p>

<ul>
  <li>Menerima input perintah dari Base Station.</li>
  <li>Mengambil video stream dari kamera HP / IP Webcam.</li>
  <li>Memanggil modul <code>vision.py</code> untuk analisis gambar.</li>
  <li>Memanggil <code>control.py</code> untuk perhitungan kendali.</li>
  <li>Mengirimkan hasil kendali ke <code>communication.py</code> supaya diteruskan ke ESP32.</li>
  <li>Meneruskan data sensor dari ESP32 kembali ke Base Station.</li>
</ul>

<p>
<code>main.py</code> adalah pusat alur: menerima data, memproses, mengirim balikan ke UI.
</p>

<hr>

<h3>vision.py (Pemrosesan Kamera)</h3>

<p>
Modul ini bertanggung jawab untuk seluruh proses komputer visi:
</p>

<ul>
  <li>Mengambil frame dari IP Webcam (HTTP/TCP).</li>
  <li>Mendeteksi garis (lane detection).</li>
  <li>Memberikan nilai <code>angle_deg</code> untuk dikirim ke kontrol.</li>
  <li>Mendeteksi obstacle menggunakan metode visual (jika ada).</li>
</ul>

<p>
Output utama ke <code>main.py</code>:
</p>

<pre>
{
  "angle": ...,
  "lane_status": ...,
  "debug_frame": ...,
  "obstacle_info": { ... }
}
</pre>

<hr>

<h3>control.py (Kendali Robot)</h3>

<p>
Modul ini mengatur logika pengendalian gerak robot berdasarkan hasil vision:
</p>

<ul>
  <li>Penerapan PID untuk smoothing dan feedback sudut.</li>
  <li>Penentuan kecepatan berdasarkan besarnya steering.</li>
  <li>Mode normal → hindar obstacle → kembali ke jalur.</li>
</ul>

<p>
Output ke <code>main.py</code>:
</p>

<pre>
{
  "steer": ...,
  "speed": ...
}
</pre>

<p>
File ini memastikan robot bergerak stabil dan aman.
</p>

<hr>

<h3>communication.py (Pengiriman & Penerimaan Data)</h3>

<p>
Modul yang menangani komunikasi jaringan:
</p>

<ul>
  <li><strong>WebSocket (TCP)</strong> untuk komunikasi dengan Base Station.</li>
  <li><strong>UDP</strong> untuk mengirim perintah motor ke ESP32.</li>
  <li>Menerima balikan sensor obstacle dari ESP32.</li>
</ul>

<p>
Data dari control.py dikirim ke ESP32 dalam format:
</p>

<pre>
"angle=<value>;speed=<value>"
</pre>

<p>
Balikan dari ESP32:
</p>

<pre>
"obs_distance=<value>"
</pre>

<hr>

<h3>ESP32 (Robot)</h3>

<ul>
  <li>Menerima perintah <code>angle</code> dan <code>speed</code> via UDP</li>
  <li>Mengatur motor DC melalui PWM</li>
  <li>Mengirim data jarak obstacle ke laptop</li>
</ul>

<p>
ESP32 (ujung output) kirim data untuk menjalankan robot
</p>

<h2>PROGRAM EXPLAIN: COMMUNICATION</h2>
<img src="komunikasi.png"></img>
<h3>HP / Kamera (IP Webcam) → Laptop</h3>
<ul>
  <li>Media: <b>WiFi</b></li>
  <li>Protocol: <b>HTTP Stream / MJPEG</b></li>
  <li>Transport: <b>TCP</b></li>
  <li>Data yang dikirim: <b>Video stream (frame)</b> untuk proses vision.</li>
</ul>

<h3>Laptop (main.py)</h3>
<p>Menjadi pusat pemrosesan seluruh data dan komando.</p>

<h4>Laptop → ESP32</h4>
<ul>
  <li>Media: <b>WiFi</b></li>
  <li>Protocol: <b>UDP</b></li>
  <li>Transport: <b>UDP</b></li>
  <li>Data dikirim:
    <ul>
      <li>Steering angle</li>
      <li>Motor speed (PWM)</li>
    </ul>
  </li>
</ul>

<h4>ESP32 → Laptop</h4>
<ul>
  <li>Protocol: <b>UDP</b></li>
  <li>Transport: <b>UDP</b></li>
  <li>Data dikirim: <b>Obstacle distance</b></li>
</ul>

<h4>Laptop ↔ Base Station / Control Panel</h4>
<ul>
  <li>Media: <b>WiFi</b></li>
  <li>Protocol: <b>WebSocket</b></li>
  <li>Transport: <b>TCP</b></li>
  <li>Data yang dikirim ke Base Station:
    <ul>
      <li>Angle</li>
      <li>Speed</li>
      <li>Lane status</li>
      <li>Robot position</li>
      <li>Obstacle info</li>
      <li>Status perintah robot</li>
    </ul>
  </li>
</ul>

<h3>ESP32</h3>
<ul>
  <li>Menerima perintah <b>angle + PWM speed</b> dari Laptop.</li>
  <li>Mengirim balik <b>jarak obstacle</b>.</li>
  <li>Menjalankan aktuator (motor DC + steering servo).</li>
</ul>

<h2>PROGRAM EXPLAIN: CONFIGURATION CONTROL</h2>
<!-- <img src="konfigurasikontrol.png"></img> -->
<h3>Input Kendali</h3>
<ul>
  <li><b>Lane Angle</b> -> dihitung dari hasil BEV + Hough transform</li>
  <li><b>Lane Status</b> -> Detected / Lost</li>
  <li><b>Robot Position</b> -> left / center / right</li>
  <li><b>Obstacle Distance</b> -> dikirim dari ESP32 via UDP</li>
</ul>

<h3>Decision Making (RobotController)</h3>
<p>Robot menggunakan finite-state logic:</p>
<ul>
  <li><b>normal</b> — mengikuti marka jalan (PID steering)</li>
  <li><b>avoid</b> — obstacle dekat → belok kanan</li>
  <li><b>return_left</b> — setelah obstacle hilang → kembali ke jalur</li>
</ul>

<h3>Output Kendali</h3>
<ul>
  <li><b>Steering Angle</b> (−45° sampai +45°)</li>
  <li><b>Speed PWM</b>, ditentukan dari kurva:
    <ul>
      <li>Sisi belok besar → PWM rendah</li>
      <li>Jalan lurus → PWM tinggi</li>
    </ul>
  </li>
</ul>

<h3>Pengiriman ke ESP32</h3>
<ul>
  <li>Media: <b>WiFi</b></li>
  <li>Protocol: <b>UDP</b></li>
  <li>Data Format: <code>steer,speed\n</code></li>
</ul>

<h3>Peran ESP32</h3>
<ul>
  <li>Menerapkan PWM ke motor DC</li>
  <li>Menggerakkan servo steering</li>
  <li>Mengirim data <b>distance (cm)</b> kembali ke Laptop</li>
</ul>

<h3>Telemetry</h3>
<p>
Semua data kendali (angle, speed, status, obstacle) dikirim ke Base Station melalui WebSocket (TCP).
</p>

<h2>ISI HATI IJUL</h2>
<p>Mff mas mesoh, ini definisi menikmati proses</p>
<h3>7 November 2025</h3>
<p>Jujur masih bingung kira-kira gimana ya hubungin program ke robotnya, takut jalan aja gx bisa njir &#128128</p>
<h3>8 November 2025</h3>
<p>Coba cicil control panel a.k.a BS dulu deh, jujur agak takut</p>
<h3>9 November 2025</h3>
<p>Tampilan udah oke, tp ngiseng e iki connection bs ku kok bosok yo</p>
<p>Mechanical baru eksekusi robotnya skrng, aku no info electric gmn, gx pernah reach out klo bkn aku yang chat duluan</p>
<p>disini aku solve sesuatu dan menambah error baru, solve sesuatu dan menambah error baru lagi</p>
<p>aku wes dapat informasi jobdesk aKu gmn</p>
<h3>10 November 2025</h3>
<p>Mulai fix bs</p>
<h3>11 November 2025</h3>
<p>Masih ada kendala buat ngonekin bs</p>
<h3>12 November 2025</h3>
<p>wah bisa jir, tp ini data framenya bermasalah</p>
<h3>13 November 2025</h3>
<p>alhamdulillah programku progress lah</p>
<p>ROBOT E DRG JADI</p>
<h3>14 November 2025</h3>
<p>okelah dl ditambah 2 hari, semoga fisik robotku selesai</p>
<h3>15 November 2025</h3>
<p>blm jadi</p>
<h3>16 November 2025</h3>
<p>blm jadi robotnya</p>
<h3>17 November 2025</h3>
<p>0 testing, hehe</p>