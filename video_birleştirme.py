import cv2
import numpy as np

# Video dosyalarının isimleri ve yolunu belirtin
video1_path = 'output_1.mp4'
video2_path = 'output_2.mp4'
video3_path = 'output_3.mp4'

# Videoları oku
video1 = cv2.VideoCapture(video1_path)
video2 = cv2.VideoCapture(video2_path)
video3 = cv2.VideoCapture(video3_path)

# Videoların genişlik ve yükseklik değerlerini al
width = int(video1.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(video1.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Yeni bir video dosyası oluştur
output_path = 'merged_video.mp4'
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
output_video = cv2.VideoWriter(output_path, fourcc, 30.0, (width, height))

# İlk videoyu yazdır
while video1.isOpened():
    ret, frame = video1.read()
    if not ret:
        break
    output_video.write(frame)

# İkinci videoyu yazdır
while video2.isOpened():
    ret, frame = video2.read()
    if not ret:
        break
    output_video.write(frame)

# Üçüncü videoyu yazdır
while video3.isOpened():
    ret, frame = video3.read()
    if not ret:
        break
    output_video.write(frame)

# Video dosyalarını kapat
video1.release()
video2.release()
video3.release()
output_video.release()

print("Videolar başarıyla birleştirildi.")
