import cv2

# Video dosyasının adını belirtin
video_path = 'merged_video.mp4'

# Videoyu açın
cap = cv2.VideoCapture(video_path)

# Video özelliklerini alın
fps = cap.get(cv2.CAP_PROP_FPS)
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

# Kesilecek süreyi belirtin (örneğin, ilk 4 dakika)
cut_duration = 4 * 60  # saniye cinsinden

# Kesilecek frame sayısını ve başlangıç frame'ini hesaplayın
cut_frame_count = int(fps * cut_duration)

# Yeni video dosyasını oluşturun
output_path = 'trimmed_video.mp4'
out = cv2.VideoWriter(output_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (int(cap.get(3)), int(cap.get(4))))

# Videoyu kesip yeni dosyaya yazmaya başlayın
frame_count = 0
while frame_count < cut_frame_count:
    ret, frame = cap.read()
    if not ret:
        break
    out.write(frame)
    frame_count += 1

# Video dosyalarını kapatın
cap.release()
out.release()

print("Video başarıyla kırpıldı. Yeni video:", output_path)
