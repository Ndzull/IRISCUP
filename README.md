# Ijul IRIS CUP Archive

<p>bismillah aja dulu</p>
<p>mfff mas yang flow program kebanyakan aku minta tolong fpt buat nulisin, ngejer DL JAM 5 20 MENIT LAGI</p>

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
<img src="konfigurasikontrol.png"></img>
<h3>Laptop</h3>
<ul>
  <li><b>main.py</b><br>
      Merupakan program utama yang menjalankan seluruh proses. File ini menjadi titik awal ketika sistem dimulai.</li>

  <li><b>vision.py (robot baca kamera)</b><br>
      Program membaca input visual dari kamera/HP, kemudian melakukan deteksi objek atau kondisi tertentu. Hasil deteksi dikirim ke modul pengambil keputusan.</li>

  <li><b>Command START/STOP</b><br>
      Berdasarkan hasil proses vision.py, sistem menghasilkan perintah seperti <i>START</i> atau <i>STOP</i> untuk mengendalikan robot.</li>

  <li><b>vision.py control.py</b><br>
      Modul ini bertanggung jawab mengubah perintah yang dihasilkan menjadi data kontrol yang siap dikirim ke ESP32.</li>

  <li><b>Base Station (Control Panel)</b><br>
      Panel kendali yang digunakan operator untuk memantau status robot dan memberikan perintah manual bila diperlukan. Informasi dari robot juga dikirim kembali ke panel ini.</li>

  <li><b>HP/Kamera</b><br>
      Digunakan untuk menyediakan input visual yang akan diproses oleh vision.py untuk mendeteksi keadaan lingkungan atau target.</li>
</ul>

<hr>

<h3>ESP32</h3>
<ul>
  <li>ESP32 menerima data kontrol dari <b>vision.py control.py</b> melalui komunikasi jaringan.</li>
  <li>Data ini kemudian diteruskan ke STM32 untuk dieksekusi pada modul motor atau sensor.</li>
</ul>

<hr>

<h3>STM32</h3>
<ul>
  <li><b>Robot baca sensor</b><br>
      STM32 membaca data sensor (ultrasonik, encoder, IMU, dll). Hasil pembacaan ini dikirim kembali ke ESP32.</li>
</ul>

<hr>

<h3>Alur utama sistem</h3>
<ol>
  <li>Kamera mengirimkan citra ke laptop.</li>
  <li><i>vision.py</i> memproses citra → menghasilkan deteksi.</li>
  <li>Hasil deteksi memicu <i>Command START/STOP</i>.</li>
  <li><i>vision.py control.py</i> mengubah perintah menjadi data kontrol.</li>
  <li>Data dikirim ke ESP32.</li>
  <li>ESP32 meneruskan ke STM32.</li>
  <li>STM32 menjalankan aktuator & membaca sensor.</li>
  <li>Data sensor kembali ke ESP32 → laptop → Base Station.</li>
</ol>

<p>Struktur ini membuat robot bisa berjalan secara otomatis maupun manual dengan pengawasan operator.</p>
<h3>Telemetry</h3>
<p>
Semua data kendali (angle, speed, status, obstacle) dikirim ke Base Station melalui WebSocket (TCP).
</p>
<h2>PROGRAM EXPLAIN: KINEMATIC CONTROL</h2>
<img src="kinematikakontrol.png"></img>
<ol>
  <li>
    <b>Input diterima dari Vision:</b><br>
    Sudut lane (<i>angle_deg</i>), status jalur, dan data obstacle 
    (<i>detected</i>, <i>distance_cm</i>).
  </li>

  <li>
    <b>Pengecekan Obstacle:</b><br>
    Jika obstacle terdeteksi & jarak < 40 cm → robot masuk <i>state = "avoid"</i> 
    dan belok kanan (steer = 45°, speed = 700).
  </li>

  <li>
    <b>State Machine Kendali:</b>
    <ul>
      <li><b>avoid</b>: terus menghindar sampai obstacle hilang atau jarak > 60 cm.</li>
      <li><b>return_left</b>: robot belok kiri untuk kembali ke jalur sampai sudut < 15°.</li>
      <li><b>normal</b>: tidak ada obstacle, lanjut pakai PID + smoothing.</li>
    </ul>
  </li>

  <li>
    <b>Smoothing Sudut (low-pass filter):</b><br>
    Sudut baru = α × sudut_baru + (1–α) × sudut_sebelumnya
  </li>

  <li>
    <b>PID Steering:</b><br>
    Sudut smooth dikirim ke PID → menghasilkan nilai steering yang lebih stabil.
  </li>

  <li>
    <b>Limit Steering:</b><br>
    Steering dibatasi pada rentang −45° hingga 45°.
  </li>

  <li>
    <b>Penentuan Kecepatan Berdasarkan Besar Steering:</b><br>
    • Sudut besar → speed rendah<br>
    • Sudut kecil → speed tinggi
  </li>

  <li>
    <b>Output Motor Command:</b><br>
    Controller mengembalikan <code>(steer, speed)</code> untuk dikirim ke motor.
  </li>
</ol>



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
