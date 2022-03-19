import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.cm as cm
from os import listdir
from os.path import isfile, join

import moviepy.editor as mpe
from moviepy.video.VideoClip import TextClip
import moviepy.video.fx as fx

trial = 0

folder = 'cyberzoo_autonomous_flight_data_set_16032022/'

runs = ['20220316-144238/', '20220316-145921/', '20220316-150634/']
run = runs[trial]

csv_runs = ['20220316-144307.csv', '20220316-145943.csv', '20220316-150902.csv']
csv_run = csv_runs[trial]

data = np.loadtxt(folder+csv_run, delimiter=',', skiprows=1, dtype=np.str)
left_diverg = np.char.rstrip(data[:, 13], '.5')
right_diverg = np.char.rstrip(data[:, 14], '.50')
time = data[:, 0].astype(float)


ims = np.array([int(im.strip('.jpg')) for im in listdir(folder+run)])
ims = np.sort(ims)
duration = np.diff(ims) / 1000000
duration = np.append(duration, duration[-1])


clips_video = []
for i, im in enumerate(ims):
    file = str(im) + '.jpg'
    clip = mpe.ImageClip(folder+run+file).set_duration(duration[i])

    clips_video.append(clip)

clips_text = []
for i, im in enumerate(ims):
    idx = (np.abs(time-im/1000000)).argmin()
    ldiv = left_diverg[idx]
    rdiv = right_diverg[idx]
    clip = TextClip(txt=f'T = {im}, ldiv = {ldiv}, rdiv={rdiv}', fontsize=50, color='white')
    clips_text.append(clip.set_position('center').set_duration(duration[i]))



concat_clip_video = mpe.concatenate_videoclips(clips_video, method="compose")
concat_clip_video = concat_clip_video.resize(4).rotate(-270)


concat_clip_text = mpe.concatenate_videoclips(clips_text, method='compose')

video = mpe.CompositeVideoClip([concat_clip_video, concat_clip_text])

video.write_videofile("test_f0.mp4", fps=24, threads=8)






