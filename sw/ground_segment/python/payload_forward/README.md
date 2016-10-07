PAYLOAD
=======

PAYLOAD (downlink) and PAYLOAD_COMMAND (uplink) messages in the pprzlink telemetry act as easy pass-through of information from an autopilot subsystem like a computer vision board to ground station payload control system.

The paparazzi autopilot does not need to understand the data, but just needs to relay it. Payload is typically highly compressed information and only piggy-backs on telemetry when it needs to be transferred over the same long-range low bitrate datalink such as the autopilot telemetry. PAYLOAD is typically used by an external application and needs to be forwarded. ```payload_forward.py``` will forward only the PAYLOAD from IVY to an IP:PORT.

The PAYLOAD sender (typically a paparazzi module) must make sure the telemetry datalink does not get flooded with payload.


Over the years some standards have developed for payload. One is sending jpeg-thumbnails-chunks. A decoder for this 'jpeg100' is added to the forwarding program. For convenience and debugging, the payload_forward program also contains some standard decoders.

Use ```payload.py --help``` to find currently supported options.
