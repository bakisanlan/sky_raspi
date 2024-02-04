import cv2

# Video dosyasını aç
video_path = 'output_3.mp4'  # Videonun dosya yolu
cap = cv2.VideoCapture(video_path)

# Yeni FPS değerini ayarla
desired_fps = 30
output_path = 'output_video3.mp4'
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Video codec (başka codec kullanabilirsiniz)
out = cv2.VideoWriter(output_path, fourcc, desired_fps, (int(cap.get(3)), int(cap.get(4))))

# Çerçeve sayacı
frame_count = 0

while True:
    # Her bir çerçeveyi al
    ret, frame = cap.read()

    # Video sona erdiyse döngüden çık
    if not ret:
        break

    # Belirli aralıklarla çerçeve kaydet
    if frame_count % int(cap.get(cv2.CAP_PROP_FPS) / desired_fps) == 0:
        out.write(frame)

    frame_count += 1

    # 'q' tuşuna basılınca döngüden çık
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Kullanılan kaynakları serbest bırak
cap.release()
out.release()
cv2.destroyAllWindows()
