#pragma once

/* Initialise the ILI9341 display and connect to the car's WiFi AP.
 * Must be called once before display_stream_start(). */
void display_stream_init(void);

/* Start the UDP video receive task.
 * Reassembles fragments, decodes JPEG → RGB565, and flushes to LCD. */
void display_stream_start(void);
