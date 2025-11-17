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