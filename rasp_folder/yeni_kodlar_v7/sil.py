from datetime import datetime

current_time = datetime.now().strftime("%H:%M:%S")
for i in range(3):
    print(current_time)