#pragma once

/* Initialise the OV2640 camera and start the WiFi AP.
 * Must be called once before camera_stream_start(). */
void camera_stream_init(void);

/* Start the UDP video streaming task.
 * Captures JPEG frames and fragments them to VIDEO_REMOTE_IP:VIDEO_UDP_PORT. */
void camera_stream_start(void);
