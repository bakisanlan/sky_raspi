import cv2

# Video dosyasını aç
video_path = 'output_1.mp4'
cap = cv2.VideoCapture(video_path)

# Yeni FPS değerini ayarla
desired_fps = 60

# Video özelliklerini al
width = int(cap.get(3))
height = int(cap.get(4))

# Yeni video dosyasını oluştur
output_path = 'output_video01_60fps.mp4'
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(output_path, fourcc, desired_fps, (width, height))

while True:
    ret, frame = cap.read()

    if not ret:
        break

    # Yeni video dosyasına çerçeveyi yaz
    out.write(frame)

    # Orijinal videoyu göster
    cv2.imshow('Original Video', frame)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC tuşuna basılınca döngüyü kır
        break

# Kullanılan kaynakları serbest bırak
cap.release()
out.release()
cv2.destroyAllWindows()
