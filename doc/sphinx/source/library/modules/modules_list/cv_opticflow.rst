.. library modules_list cv_opticflow

======================
CV Opticflow
======================

The optical flow module in Paparazzi allows for the extraction of flow information from a video thread. 
This is particularly useful to extrapolate the velocity of the vehicle with respect to the ground or the environment.

Pipeline Overview
=================

The optical flow pipeline can be summarized as follows:

* The opticflow_module.c script looks at up to two different video threads
* The opticflow_calculator.c script runs a set of pre-selectable algorithms to extract the flow messages
* The opticflow_module.c script publishes the ABI message OPTICAL_FLOW for later processing

Usage
=================

The opticalflow module requires a correct tuning of parameters in order to perform efficiently in terms of execution speed and accuracy.

Here a set of advice is reported for a pipeline operating with:

* ACT_FAST algorithm for corner detection
* LUKAS_KANADE algorithm for feature tracking

For proper operation, keep in mind that:

* ask for frames that you will use: if your camera sends more FPS than what the OPTICFLOW_FPS parameter specifies, the pipeline slows down. Tune both the FPS of the camera driver and the FPS of the module accordingly.
* find the sweet spot between resolution and execution time: the LUKAS_KANADE algorithm requires convolution over the images to execute; this means that the speed of the process is affected by the resolution of the convoluted image. It is therefore not useful to run with high resolution images if lower resolution ones provide a good result as well.
* if you decide to use the Lukas Kanade algorithm, the PYRAMID_LEVEL parameter needs to be accurately selected: the higher, the slower the process. Sometimes a high level is not required; it depends on the task. You can find information on the meaning of this parameter here http://robots.stanford.edu/cs223b04/algo_tracking.pdf.