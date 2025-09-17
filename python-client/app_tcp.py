# %%
import matplotlib
from pathlib import Path
import socket
import struct
from time import perf_counter_ns, sleep, time
from typing import Dict, Optional
from matplotlib.animation import FuncAnimation
from matplotlib.axes import Axes
from matplotlib.gridspec import GridSpec
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from collections import deque
from netCDF4 import Dataset
from datetime import datetime
from matplotlib.widgets import Button
import threading
from queue import Queue, Empty
from tcp_thread import TcpThread
import warnings

# Ignore matplotlib warnings
warnings.filterwarnings("ignore", category=RuntimeWarning)

matplotlib.use('QtAgg')  # Use TkAgg backend for interactive plotting
# %%


class DataBuffer:
    def __init__(self, maxlen=2000):
        # Ensure maxlen is a power of 2
        maxlen = 1 << int(np.ceil(np.log2(maxlen)))
        self._data = deque(maxlen=maxlen)

    def append(self, item):
        self._data.append(item)

    def clear(self):
        self._data.clear()

    def __getitem__(self, index):
        return self._data[index]

    def __len__(self):
        return len(self._data)

    def to_dataframe(self, columns=None):
        if columns is None:
            columns = ['tstamp', 'x', 'y', 'z', 'dx', 'dy', 'dz']
        return pd.DataFrame(list(self._data), columns=columns)


class DataRate:
    def __init__(self, update_rate: float = 2.0):
        self.count = 0
        self.last = None  # Last timestamp for calculating data rate
        self.update_rate = update_rate

    def update(self, num_samples: int = 1):
        now = perf_counter_ns()
        self.count += num_samples
        if self.last is None:
            self.last = now
            return
        elapsed = (now - self.last) / 1e9
        if elapsed > 2.0:  # Update every second
            rate = self.count * 8 / elapsed
            self.last = now
            self.count = 0
            unit = 'bps'
            if rate > 1024:
                rate /= 1024
                unit = 'Kbps'
            elif rate > 1024*1024:
                rate /= 1024*1024
                unit = 'Mbps'
            print(f'Data rate: {rate:.2f} {unit}')


# %%
DPI = 72
FIG_WID = 800 / DPI
FIG_HEI = 400 / DPI


def run(queue: Queue, winsize: int = 1000):
    plt.ioff()
    grid = GridSpec(2, 8, width_ratios=[1]*8, height_ratios=[
                    0.1, 1], left=0.065, bottom=0.065, wspace=3)
    fig = plt.figure(figsize=(FIG_WID, FIG_HEI), dpi=DPI, animated=True)
    text_ax = fig.add_subplot(grid[0, :])
    text_ax.set_axis_off()
    curtime = text_ax.text(
        0.5, 0.5, "Accelerometer Data: Waiting for data...",
        fontsize=14, ha='center', va='center'
    )
    # button_ax.set_axis_off()
    axs = []
    for i in range(2):
        ar = slice(i*4, (i+1)*4)
        if i > 0:
            ax = fig.add_subplot(grid[1, ar], sharex=axs[0])
        else:
            ax = fig.add_subplot(grid[1, ar])
        ax.autoscale(enable=True, axis='y')
        axs.append(ax)
    axs = np.asarray(axs)
    for (ax, title, unit) in zip(axs, ('Acceleration', 'ω'), ('g', '°/s')):
        ax: Axes = ax
        ax.set_title(title, fontsize=10, fontweight='bold')
        ax.set_ylabel(unit, fontsize=10)
        ax.set_xlabel('Offset (ms)', fontsize=10)

    lines = []
    for mid, axm in enumerate(axs):
        lines.append([])
        lines[mid].append(
            axm.plot([], [], label='X', color='red', alpha=0.5)[0])
        lines[mid].append(
            axm.plot([], [], label='Y', color='green', alpha=0.5)[0])
        lines[mid].append(
            axm.plot([], [], label='Z', color='blue', alpha=0.5)[0])
    # fig.tight_layout()

    fig.show()

    artists = lines + [curtime]

    def update(frame):
        try:
            df = None
            while True:
                try:
                    df = queue.get_nowait()
                except Empty:
                    break
            if df is None:
                return artists
            if df.empty:
                print(f"No data available")
                return artists
            now = df['tstamp'].iloc[-1]
            sel = df['tstamp'] > (now - winsize * 1e-3)
            tstamp = df['tstamp'][sel]  # Convert to milliseconds
            curtime.set_text(f"Accelerometer Data: {now:.2f} s")
            # tstamp = tstamp * 1e-6  # Convert to seconds
            tstamp -= tstamp.iloc[-1]
            tstamp *= 1e3  # Convert to milliseconds for plotting
            for aid, (lline, ax) in enumerate(zip(lines, axs)):
                lline: list = lline
                ax: Axes = ax
                if aid % 2 == 0:
                    lline[0].set_data(tstamp, df['ax'][sel])
                    lline[1].set_data(tstamp, df['ay'][sel])
                    lline[2].set_data(tstamp, df['az'][sel])
                else:
                    lline[0].set_data(tstamp, df['wx'][sel])
                    lline[1].set_data(tstamp, df['wy'][sel])
                    lline[2].set_data(tstamp, df['wz'][sel])
                ax.relim()
                ax.autoscale_view()
        except Empty:
            sleep(0.01)
        return artists

    animation = FuncAnimation(
        fig, update, blit=False,
        repeat=False, save_count=100,
        interval=100
    )
    plt.show()
    print("Done receiving data")
    print("NetCDF file closed")
    plt.ion()


# %%
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
        description="TCP Client for Accelerometer Data")
    parser.add_argument(
        'host', type=str, help='Host address of the TCP server', default='localhost', nargs='?')
    parser.add_argument(
        'port', type=int, help='Port number of the TCP server', default=14389, nargs='?')
    parser.add_argument(
        '--window', type=int, default=1, help='Window size for data display in seconds'
    )
    args = parser.parse_args()
    winsize = args.window*1000
    if winsize < 1000:
        print(f"Window size {winsize} ms is too small, setting to 1000 ms")
        winsize = 1000
    elif winsize > 10000:
        print(f"Window size {winsize} ms is too large, setting to 10000 ms")
        winsize = 10000
    queue = Queue()
    thread = TcpThread(args.host, args.port, queue, datasize=winsize)
    thread.daemon = True  # Ensure the thread exits when the main program exits
    thread.start()
    print(
        f"Starting TCP client thread for {args.host}:{args.port} with window size {winsize} ms")
    run(queue, winsize=winsize)
