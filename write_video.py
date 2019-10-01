"""write_video.py: Writes PyGame frames to a video."""
__author__ = "Gavin Uberti"

import os
import subprocess as sp
import time

os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "hide"
from pygame import surfarray
import numpy as np

PATH = "C:\\Users\\guberti\\Downloads\\ffmpeg-20190930-6ca3d34-win64-static\\ffmpeg-20190930-6ca3d34-win64-static\\bin\\ffmpeg.exe"

class FramePipeline:
    def __init__(self, filename, size=(720, 720), framerate=24):
        command = [PATH,
            '-y', # Overwrite existing file
            '-f', 'rawvideo',
            '-vcodec', 'rawvideo',
            '-pix_fmt', 'rgb24',
            '-s', '{}x{}'.format(*size),
            '-r', str(framerate), # FPS
            '-i', '-', # Read from STDIN
            '-an', # No audio
            '-vcodec', 'libx264',
            '-preset', 'ultrafast',
            filename ]
        self.pipeline = sp.Popen(command, stdin=sp.PIPE, stderr=sp.STDOUT)

    def write_surface(self, surface):
        data = surfarray.array3d(surface)
        data = data.swapaxes(0,1)
        self.write_numpy(data)

    def write_numpy(self, nparr):
        self.pipeline.stdin.write(nparr.tostring())

    def close(self):
        self.pipeline.stdin.close()
        self.pipeline.wait()

# For testing
def main():
    pipeline = FramePipeline("test_out.mp4")
    for l in range(0, 5 * 24):
        arr = np.random.randint(255, size=(720, 720, 3), dtype=np.uint8)
        pipeline.write_numpy(arr)
    pipeline.close()

if __name__ == "__main__":
    main()
