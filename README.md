# SmileDrawer ROS2

Aplikasi ROS2 yang menggunakan beberapa turtle dalam turtlesim untuk menggambar wajah tersenyum. Program ini menggunakan tiga turtle untuk menggambar dua mata dan mulut secara terkoordinasi.

![Screenshot 2024-11-03 110346](https://github.com/user-attachments/assets/3b17f887-d33f-4d91-8346-7fd5dbd71879)

## Gambaran Umum

SmileDrawer mendemonstrasikan penggunaan:
- Multiple turtle dalam turtlesim
- Publisher dan service ROS2
- Implementasi state machine
- Kontrol gerakan terkoordinasi

## Prasyarat

- ROS2 (Diuji pada Humble)
- Python 3.x
- Paket turtlesim

## Instalasi

1. Buat workspace ROS2 (jika belum ada):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Klon repositori ini:
```bash
git clone https://github.com/namaanda/smile_drawer.git
```

3. Build workspace:
```bash
cd ~/ros2_ws
colcon build
```

4. Source file setup:
```bash
source install/setup.bash
```

## Posisi Code

```
smile_drawer/
├── smile_drawer/
│   ├── __init__.py
│   └── smile_drawer_node.py
├── setup.py
├── setup.cfg
├── package.xml
```

## Implementasi Kode

Kode utama mengikuti tiga fase utama:

1. **Inisialisasi Node dan Publisher**
```python
class SmileDrawer(Node):
    def __init__(self):
        super().__init__('smile_drawer')
        self.publisher_1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.spawn_client = self.create_client(Spawn, 'spawn')
```

2. **Persiapan dan Pembuatan Turtle**
```python
# Posisikan turtle1
self.move_to_position(self.publisher_1, 3.0, 8.0)

# Buat turtle baru
self.spawn_turtle('turtle2', 8.0, 8.0, 0.0)
self.spawn_turtle('turtle3', 5.5, 5.0, 0.0)

# Siapkan publisher
self.publisher_2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
self.publisher_3 = self.create_publisher(Twist, '/turtle3/cmd_vel', 10)
```

3. **Implementasi Logika Penggambaran**
```python
def draw_smile(self):
    if self.step == 0:
        # Gambar mata kiri
        self.move_turtle(self.publisher_1, 0.5, 2.0)
    elif self.step == 1:
        # Gambar mata kanan
        self.move_turtle(self.publisher_2, 0.5, 2.0)
    elif self.step == 2:
        # Gambar mulut
        self.move_turtle(self.publisher_3, 0.8, 0.8)
    else:
        # Selesai
        self.timer.cancel()
```

## Cara Penggunaan

1. Mulai lingkungan ROS2:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

2. Jalankan turtlesim:
```bash
ros2 run turtlesim turtlesim_node
```

3. Di terminal baru, jalankan node SmileDrawer:
```bash
ros2 run smile_drawer smile_drawer_node
```

## Parameter

- Posisi mata:
  - Mata kiri: (3.0, 8.0)
  - Mata kanan: (8.0, 8.0)
- Posisi mulut: (5.5, 5.0)
- Kecepatan gerakan:
  - Mata: linear = 0.5, angular = 2.0
  - Mulut: linear = 0.8, angular = 0.8


## Ucapan Terima Kasih

- Terima kasih kepada komunitas ROS2 dan turtlesim
- Terinspirasi dari tutorial dan contoh ROS2
