import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.cm as cm
from os import listdir
from os.path import isfile, join

import moviepy.editor as mpe

folder = 'cyberzoo_autonomous_flight_data_set_16032022/'

runs = ['20220316-144238/', '20220316-145921/', '20220316-150634/']
run = runs[2]

fig, ax = plt.subplots()

ims = np.array([int(im.strip('.jpg')) for im in listdir(folder+run)])
ims = np.sort(ims)
duration = np.diff(ims) / 1000000
duration = np.append(duration, duration[-1])


clips = []
for i, im in enumerate(ims):
    file = str(im) + '.jpg'
    clips.append(mpe.ImageClip(folder+run+file).set_duration(duration[i]))

concat_clip = mpe.concatenate_videoclips(clips, method="compose")
concat_clip.write_videofile("flight_3_m16.mp4", fps=24, threads=8)






