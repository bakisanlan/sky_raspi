import cv2

# Video dosyasının adı ve yolu
video_path = 'merged_video.mp4'

# Yeni video süresi (saniye cinsinden)
new_duration = 240  # 4 dakika = 240 saniye

# Videoyu aç
video = cv2.VideoCapture(video_path)

# Video özelliklerini al
width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = video.get(cv2.CAP_PROP_FPS)
total_frames = int(video.get(cv2.CAP_PROP_FRAME_COUNT))

# Hedeflenen kare sayısı
target_frames = int(fps * new_duration)

# Yeni bir video dosyası oluştur
output_path = 'trimmed_video.mp4'
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
output_video = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

# Belirli süreye kadar olan kırpılmış videoyu oluştur
for i in range(target_frames):
    ret, frame = video.read()
    if not ret:
        break
    output_video.write(frame)

# Video dosyalarını kapat
video.release()
output_video.release()

print("Video başarıyla kırpıldı.")
