import cv2

videonamelist = ['video_kaydı.mp4', 'video_kaydı4.mp4']

video1 = cv2.VideoCapture(videonamelist[0])
frame_width = int(video1.get(3))  
frame_height = int(video1.get(4))  
frame_rate = int(video1.get(5))  


output = cv2.VideoWriter('output2.mp4', cv2.VideoWriter_fourcc(*'mp4v'), frame_rate, (frame_width, frame_height))

class VideoMerger:
    def __init__(self, output_writer):
        self.output_writer = output_writer

    def merge_videos(self, video_list):
        for video_name in video_list:
            video = cv2.VideoCapture(video_name)
            
            while True:
                ret, frame = video.read()
                if not ret:
                    break
                
                self.output_writer.write(frame)

    def done(self):
        self.output_writer.release()

merger = VideoMerger(output)
merger.merge_videos(videonamelist)
merger.done()
