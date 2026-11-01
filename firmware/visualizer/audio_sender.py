# pip install numpy soundcard pyserial
#
#   python audio_sender.py            real audio
#   python audio_sender.py --demo     for testing the display
#   python audio_sender.py --list     show serial ports and speakers
#   python audio_sender.py --port COM5

import argparse
import math
import sys
import time

import numpy as np

COLS = 32
SAMPLE_RATE = 48000
FFT_SIZE = 2048
HOP = 1024
F_MIN = 40.0
F_MAX = 16000.0
FLOOR_DB = -60.0
TILT_DB = 10.0
FALL = 0.80
AGC_DECAY = 0.995
AGC_MIN = 0.25
HEADER = 0xFF


def band_edges():
    return np.geomspace(F_MIN, F_MAX, COLS + 1)


class Spectrum:
    def __init__(self, sample_rate=SAMPLE_RATE):
        self.window = np.hanning(FFT_SIZE)
        self.scale = 2.0 / self.window.sum()
        self.freqs = np.fft.rfftfreq(FFT_SIZE, 1.0 / sample_rate)
        self.edges = band_edges()
        self.centers = np.sqrt(self.edges[:-1] * self.edges[1:])
        self.tilt = np.linspace(0.0, TILT_DB, COLS)
        self.buffer = np.zeros(FFT_SIZE, dtype=np.float32)
        self.smooth = np.zeros(COLS)
        self.ref = AGC_MIN

    def push(self, mono):
        n = len(mono)
        self.buffer = np.concatenate([self.buffer[n:], mono.astype(np.float32)])
        spec = np.abs(np.fft.rfft(self.buffer * self.window)) * self.scale

        mags = np.empty(COLS)
        for i in range(COLS):
            inside = (self.freqs >= self.edges[i]) & (self.freqs < self.edges[i + 1])
            if inside.any():
                mags[i] = spec[inside].max()
            else:
                mags[i] = np.interp(self.centers[i], self.freqs, spec)

        db = 20.0 * np.log10(mags + 1e-9) + self.tilt
        level = np.clip((db - FLOOR_DB) / (-FLOOR_DB), 0.0, 1.0)

        self.ref = max(self.ref * AGC_DECAY, level.max(), AGC_MIN)
        level = np.clip(level / self.ref, 0.0, 1.0)

        self.smooth = np.maximum(level, self.smooth * FALL)
        return np.clip(np.round(self.smooth * 8), 0, 8).astype(np.uint8)


def frame_bytes(heights):
    return bytes([HEADER]) + bytes(int(h) for h in heights)


def find_port(wanted=None):
    import serial.tools.list_ports as lp
    if wanted:
        return wanted
    for p in lp.comports():
        if p.vid in (0x2886, 0x2E8A):
            return p.device
    return None


def list_stuff():
    import serial.tools.list_ports as lp
    print("serial ports:")
    for p in lp.comports():
        print("  ", p.device, "-", p.description)
    try:
        import soundcard as sc
        print("speakers:")
        for s in sc.all_speakers():
            print("  ", s.name)
    except Exception as e:
        print("no audio available:", e)


def audio_blocks():
    import soundcard as sc
    speaker = sc.default_speaker()
    mic = sc.get_microphone(id=str(speaker.name), include_loopback=True)
    print("listening to:", speaker.name)
    with mic.recorder(samplerate=SAMPLE_RATE, channels=2, blocksize=HOP) as rec:
        while True:
            data = rec.record(numframes=HOP)
            yield data.mean(axis=1)


def demo_heights():
    start = time.time()
    while True:
        t = time.time() - start
        yield np.array(
            [max(0, min(8, int(round(4 + 4 * math.sin(2 * t + i * 0.35))))) for i in range(COLS)],
            dtype=np.uint8,
        )
        time.sleep(1 / 30)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port")
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()

    if args.list:
        list_stuff()
        return

    import serial

    port = find_port(args.port)
    if port is None:
        print("couldn't find the XIAO, try --list and then --port COMx")
        sys.exit(1)
    print("connecting to", port)
    ser = serial.Serial(port, 115200, write_timeout=1)
    time.sleep(1.5)  

    try:
        if args.demo:
            for heights in demo_heights():
                ser.write(frame_bytes(heights))
        else:
            spectrum = Spectrum()
            for mono in audio_blocks():
                ser.write(frame_bytes(spectrum.push(mono)))
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
