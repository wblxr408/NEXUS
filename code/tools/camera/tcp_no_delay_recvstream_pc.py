import cv2
import numpy as np
import subprocess
from multiprocessing import Array

def video_display(ffmpeg_command, frame_array, W_img, H_img):
    process = subprocess.Popen(ffmpeg_command, stdout=subprocess.PIPE)
    
    while True:
        raw_frame = process.stdout.read(W_img * H_img * 3)  # RGB 3 channels
        if not raw_frame:
            break

        # 将帧复制到共享内存中
        frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape(H_img, W_img, 3)
        np.copyto(np.frombuffer(frame_array.get_obj(), dtype=np.uint8).reshape(H_img, W_img, 3), frame)

        # 显示帧
        cv2.imshow('Real-Time Video', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    process.terminate()

if __name__ == '__main__':
    W_img = 640
    H_img = 480

    shared_frame = Array('B', W_img * H_img * 3)  # RGB 3通道，大小为像素数 × 3

    # 使用 ffmpeg 读取流并输出为 RGB 格式
    ffmpeg_command = [
        'ffplay',
        '-i', 'tcp://192.168.0.112:5005',
        '-vf', 'setpts=N/30',
        '-fflags', 'nobuffer',
        '-flags', 'low_delay',
        '-framedrop'
    ]

    video_display(ffmpeg_command, shared_frame, W_img, H_img)
