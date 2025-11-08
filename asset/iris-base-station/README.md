# IRIS Base Station - Lane Detection Monitor

Base Station untuk monitoring dan visualisasi data lane detection dari program vision processing. Mendukung koneksi via WebSocket standar atau ROS (roslibjs).

## 📋 Deskripsi

Base Station ini dibuat untuk Final Project Magang Tim IRIS. Aplikasi ini menerima data real-time dari program vision processing yang mendeteksi jalur menggunakan OpenCV, termasuk:

- **Video Stream**: Raw camera feed dan hasil threshold/processing
- **Telemetry Data**: Speed kendaraan, angle, dan jarak tempuh
- **Optional Data**: Deteksi obstacle, jarak, dan posisi

## 🚀 Fitur

### 1. Dual Connection Mode

- **Standard WebSocket**: Untuk program C++/Python biasa
- **ROS (roslibjs)**: Untuk program yang menggunakan ROS 2

### 2. Real-time Visualization

- Dual video display (raw & processed)
- FPS counter untuk setiap stream
- Resolution info

### 3. Telemetry Dashboard

- Speed gauge dengan visualisasi
- Lane detection status
- Angle (sudut steering/kemiringan jalur)
- Jarak tempuh (total distance traveled)
- Obstacle detection (optional)

### 4. System Monitoring

- Connection status indicator
- Real-time system logs
- Timestamp untuk setiap event

## 🛠️ Teknologi

- **HTML5**: Struktur aplikasi
- **Tailwind CSS**: Styling dengan dark mode (Vercel/ShadCN style)
- **Vanilla JavaScript**: Logic aplikasi (ES6+)
- **Canvas API**: Rendering video streams
- **WebSocket API**: Real-time communication
- **roslibjs**: ROS bridge integration

## 📦 Instalasi

### Cara 1: Langsung Buka di Browser

1. Download semua file
2. Pastikan struktur folder:
   ```
   iris-base-station/
   ├── index.html
   ├── app.js
   ├── logo.svg
   └── README.md
   ```
3. Buka `index.html` di browser modern (Chrome, Firefox, Edge)

### Cara 2: Menggunakan Local Server (Recommended)

```bash

# Menggunakan Python

python -m http.server 8000

# Atau menggunakan Node.js

npx serve

# Atau menggunakan PHP

php -S localhost:8000
```

Kemudian buka `http://localhost:8000` di browser.

## 🔌 Koneksi

### Mode 1: Standard WebSocket

Untuk program C++/Python yang menggunakan WebSocket biasa.

**Server URL**: `ws://localhost:8080` (sesuaikan dengan server Anda)

#### Format Pesan WebSocket

Kirim data dalam format JSON:

```json
// Image Raw
{
"type": "image_raw",
"data": "base64_encoded_image_data",
"width": 640,
"height": 480
}

// Image Processed
{
"type": "image_processed",
"data": "base64_encoded_image_data",
"width": 640,
"height": 480
}

// Telemetry (Data Utama yang Wajib Dikirim)
{
"type": "telemetry",
"data": {
"speed": 25.5, // Speed dalam cm/s (opsional)
"laneStatus": "Detected", // Status: "Detected", "Partial", "Lost" (wajib)
"angle": 15.5, // Sudut steering dalam derajat (wajib)
"jarakTempuh": 125.3 // Jarak tempuh dalam meter (wajib)
}
}

// Obstacle (Optional - hanya jika ada fitur deteksi obstacle)
{
"type": "obstacle",
"detected": true,
"distance": 250,
"position": "center"
}
```

#### Data yang Wajib Dikirim

Base Station membutuhkan data berikut untuk ditampilkan:

1. **Image Raw** (`type: "image_raw"`)

   - Data gambar mentah dari kamera
   - Format: Base64 encoded JPEG/PNG
   - Include width dan height

2. **Image Processed** (`type: "image_processed"`)

   - Data gambar hasil threshold/processing
   - Format: Base64 encoded JPEG/PNG
   - Include width dan height

3. **Telemetry** (`type: "telemetry"`)

   - **speed**: Kecepatan kendaraan (cm/s)
   - **laneStatus**: Status deteksi jalur ("Detected", "Partial", "Lost")
   - **angle**: Sudut steering/kemiringan jalur (derajat, -90 hingga +90)
   - **jarakTempuh**: Total jarak yang sudah ditempuh (meter)

4. **Obstacle** (OPTIONAL - `type: "obstacle"`)
   - Hanya kirim jika program kalian punya fitur deteksi obstacle
   - **detected**: boolean (true/false)
   - **distance**: jarak ke obstacle (centimeter)
   - **position**: posisi obstacle ("left", "center", "right")

#### Contoh Python Client (WebSocket)

```python
import asyncio
import websockets
import json
import cv2
import base64

async def send_data():
uri = "ws://localhost:8080"
async with websockets.connect(uri) as websocket:
cap = cv2.VideoCapture(0)
jarak_tempuh = 0.0

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # Encode image to base64
            _, buffer = cv2.imencode('.jpg', frame)
            img_base64 = base64.b64encode(buffer).decode('utf-8')

            # Send raw image
            await websocket.send(json.dumps({
                "type": "image_raw",
                "data": img_base64,
                "width": frame.shape[1],
                "height": frame.shape[0]
            }))

            # Process image (example: threshold)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

            # Send processed image
            _, buffer_proc = cv2.imencode('.jpg', thresh)
            img_proc_base64 = base64.b64encode(buffer_proc).decode('utf-8')

            await websocket.send(json.dumps({
                "type": "image_processed",
                "data": img_proc_base64,
                "width": thresh.shape[1],
                "height": thresh.shape[0]
            }))

            # Calculate metrics
            speed = calculate_speed()  # cm/s
            angle = calculate_angle(thresh)  # degrees
            jarak_tempuh += speed * 0.033 / 100  # update distance (cm/s * time / 100 = meter)

            # Send telemetry (DATA WAJIB)
            await websocket.send(json.dumps({
                "type": "telemetry",
                "data": {
                    "speed": speed,              # cm/s
                    "angle": angle,              # degrees
                    "jarakTempuh": jarak_tempuh, # meter
                    "laneStatus": "Detected"     # "Detected", "Partial", or "Lost"
                }
            }))

            # Optional: Send obstacle data (jika ada)
            # await websocket.send(json.dumps({
            #     "type": "obstacle",
            #     "detected": False,
            #     "distance": 0,
            #     "position": "center"
            # }))

            await asyncio.sleep(0.033)  # ~30 FPS

asyncio.run(send_data())
```

#### Contoh C++ Client (WebSocket)

Gunakan library seperti `websocketpp` atau `Boost.Beast`:

```cpp
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
typedef websocketpp::client<websocketpp::config::asio_client> client;

int main() {
client c;
websocketpp::connection_hdl hdl;

    // Initialize WebSocket connection
    // ... (connection setup code)

    cv::VideoCapture cap(0);
    cv::Mat frame, processed;
    double jarak_tempuh = 0.0;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // Encode to base64
        std::vector<uchar> buf;
        cv::imencode(".jpg", frame, buf);
        std::string img_base64 = base64_encode(buf.data(), buf.size());

        // Send raw image
        json msg_raw = {
            {"type", "image_raw"},
            {"data", img_base64},
            {"width", frame.cols},
            {"height", frame.rows}
        };
        c.send(hdl, msg_raw.dump(), websocketpp::frame::opcode::text);

        // Process and calculate metrics
        double speed = calculateSpeed();
        double angle = calculateAngle(processed);
        jarak_tempuh += speed * 0.033 / 100; // update distance (cm/s * time / 100 = meter)

        // Send telemetry
        json msg_telemetry = {
            {"type", "telemetry"},
            {"data", {
                {"speed", speed},
                {"angle", angle},
                {"jarakTempuh", jarak_tempuh},
                {"laneStatus", "Detected"}
            }}
        };
        c.send(hdl, msg_telemetry.dump(), websocketpp::frame::opcode::text);
    }

    return 0;

}
```

### Mode 2: ROS (roslibjs)

Untuk program yang menggunakan ROS 2.

**Server URL**: `ws://localhost:9090` (ROS bridge default)

#### Setup ROS Bridge

```bash

# Install rosbridge_suite

sudo apt install ros-<distro>-rosbridge-suite

# Run rosbridge

ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

#### ROS Topics

Configure topics di Base Station UI:

- **Image Raw Topic**: `/camera/image_raw` (sensor_msgs/CompressedImage)
- **Image Processed Topic**: `/camera/image_processed` (sensor_msgs/CompressedImage)
- **Speed Topic**: `/vehicle/speed` (std_msgs/Float32)

#### Contoh ROS 2 Publisher (Python)

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
import cv2

class LaneDetectionNode(Node):
def **init**(self):
super().**init**('lane_detection_node')

        # Publishers
        self.pub_raw = self.create_publisher(
            CompressedImage, '/camera/image_raw', 10)
        self.pub_processed = self.create_publisher(
            CompressedImage, '/camera/image_processed', 10)
        self.pub_speed = self.create_publisher(
            Float32, '/vehicle/speed', 10)

        # Timer
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.cap = cv2.VideoCapture(0)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # Publish raw image
        msg_raw = CompressedImage()
        msg_raw.header.stamp = self.get_clock().now().to_msg()
        msg_raw.format = "jpeg"
        msg_raw.data = cv2.imencode('.jpg', frame)[1].tobytes()
        self.pub_raw.publish(msg_raw)

        # Process image
        processed = self.process_image(frame)

        # Publish processed image
        msg_proc = CompressedImage()
        msg_proc.header.stamp = self.get_clock().now().to_msg()
        msg_proc.format = "jpeg"
        msg_proc.data = cv2.imencode('.jpg', processed)[1].tobytes()
        self.pub_processed.publish(msg_proc)

        # Calculate and publish speed
        speed = self.calculate_speed()
        msg_speed = Float32()
        msg_speed.data = float(speed)
        self.pub_speed.publish(msg_speed)

    def process_image(self, frame):
        # Your lane detection logic here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        return thresh

    def calculate_speed(self):
        # Your speed calculation logic here
        return 0.0

def main():
rclpy.init()
node = LaneDetectionNode()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()

if **name** == '**main**':
main()
```

## 📊 Data Format

### Speed

- **Unit**: cm/s (centimeter per second)
- **Range**: 0 hingga maksimal kendaraan
- Kecepatan pergerakan kendaraan

### Angle

- **Unit**: derajat (°)
- **Range**: -90° (kiri maksimal) hingga +90° (kanan maksimal)
- **0°**: Lurus
- **Positif**: Belok kanan
- **Negatif**: Belok kiri
- Sudut steering atau kemiringan jalur yang terdeteksi

### Jarak Tempuh

- **Unit**: meter (m)
- Akumulasi total jarak yang sudah ditempuh kendaraan
- Dihitung dari: `jarak_tempuh += speed * delta_time / 100`

### Lane Status

- `"Detected"`: Jalur terdeteksi dengan baik
- `"Partial"`: Jalur terdeteksi sebagian
- `"Lost"`: Jalur tidak terdeteksi

### Obstacle (Optional)

Hanya perlu dikirim jika program kalian memiliki fitur deteksi obstacle:

- **detected**: `true` atau `false`
- **distance**: Jarak ke obstacle dalam centimeter
- **position**:
  - `"left"`: Obstacle di kiri
  - `"center"`: Obstacle di tengah
  - \*\*"right"`: Obstacle di kanan

## 🎨 Customization

### Mengubah Warna/Theme

Edit bagian `tailwind.config` di `index.html`:

```javascript
tailwind.config = {
theme: {
extend: {
colors: {
// Customize colors here
primary: { ... },
secondary: { ... }
}
}
}
}
```

### Menambah Metric Baru

1. Tambahkan UI element di `index.html`
2. Update `updateTelemetry()` function di `app.js`
3. Kirim data dari program vision processing

## 🐛 Troubleshooting

### WebSocket tidak bisa connect

- Pastikan server WebSocket sudah running
- Check firewall settings
- Pastikan URL dan port sudah benar

### ROS tidak bisa connect

- Pastikan rosbridge sudah running: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
- Check ROS_DOMAIN_ID
- Pastikan topic names sudah benar

### Video tidak muncul

- Check format base64 image
- Pastikan image data valid
- Check browser console untuk error

### FPS rendah

- Reduce image resolution
- Optimize processing algorithm
- Check network latency

## 📝 Best Practices

### Untuk Anak Magang

1. **Mulai dengan WebSocket Sederhana**

   - Lebih mudah untuk debugging
   - Tidak perlu setup ROS dulu

2. **Test Incremental**

   - Test kirim image raw dulu
   - Kemudian tambah image processed
   - Lalu tambah telemetry data (speed, angle, jarakTempuh, laneStatus)
   - Terakhir tambah optional features (obstacle detection)

3. **Fokus ke Data Wajib Dulu**

   - Pastikan 4 data wajib terkirim dengan benar:
     1. Image raw
     2. Image processed
     3. Speed
     4. Lane status
     5. Angle
     6. Jarak tempuh
   - Obstacle detection adalah OPTIONAL (bonus)

4. **Optimize Image Size**

   - Resize image sebelum kirim (misal 640x480)
   - Gunakan JPEG compression
   - Jangan kirim raw bitmap

5. **Error Handling**

   - Tambahkan try-catch di semua fungsi
   - Log error untuk debugging
   - Reconnect otomatis jika disconnect

6. **Code Organization**
   - Pisahkan fungsi capture, process, dan send
   - Gunakan multiple nodes/programs (nilai plus!)
   - Comment code dengan baik

## 🎯 Checklist Final Project

Pastikan program kalian bisa:

- [ ] Capture video dari IP camera via WiFi
- [ ] Kirim raw image ke Base Station
- [ ] Process image (threshold/lane detection)
- [ ] Kirim processed image ke Base Station
- [ ] Hitung dan kirim speed (cm/s) (Opsional)
- [ ] Hitung dan kirim angle (derajat)
- [ ] Hitung dan kirim jarak tempuh (meter)
- [ ] Kirim lane status ("Detected", "Partial", "Lost", terserah....)
- [ ] (Optional) Deteksi dan kirim data obstacle

## 📚 Resources

- [WebSocket API Documentation](https://developer.mozilla.org/en-US/docs/Web/API/WebSocket)
- [roslibjs Documentation](http://robotwebtools.org/jsdoc/roslibjs/current/)
- [OpenCV Python Tutorials](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)

## 🤝 Support

Jika ada pertanyaan atau masalah:

1. Check troubleshooting section
2. Review example code
3. Tanya ke bengkel

## 📄 License

Project ini dibuat untuk keperluan internal Tim IRIS.

---

**Good luck dengan Final Project kalian! 🚀**

_Made with ❤️ untuk Anak Magang Tim IRIS_
