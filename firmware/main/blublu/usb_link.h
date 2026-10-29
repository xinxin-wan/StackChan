/*
 * BluBlu peripheral mode: framed request/response link over the chip's built-in
 * USB-Serial-JTAG (CDC-ACM, VID:PID 303a:1001). No network interface is created,
 * so the host network is not touched. Works with Android (UsbManager), Linux
 * (cdc_acm, /dev/ttyACM*) and Windows (usbser, COMx) without extra drivers.
 *
 * Frame (all integers little endian):
 *   0  2  magic "SC"
 *   2  1  version (1)
 *   3  1  type: 1 = request (host->device), 2 = response, 3 = event (device->host)
 *   4  2  seq (response echoes the request seq; events use 0;
 *          a request with seq 0 gets no response)
 *   6  4  payload length
 *   10 2  low 16 bits of CRC-32 over bytes 0..9
 *   12 n  payload: u32 json_len | JSON object (UTF-8) | binary (e.g. JPEG)
 *   .. 4  CRC-32 (zlib) over the payload
 *
 * Stream events carry the stream id returned by "stream.start" ({"stream": id}),
 * so events of a previous stream can be told apart.
 *
 * Hosts must not assert RTS (USB-Serial-JTAG resets the chip on RTS) and must
 * resync on the magic: ROM / bootloader text and stale bytes may precede frames.
 */
#pragma once

namespace blublu::usb_link {

bool start();

}  // namespace blublu::usb_link
