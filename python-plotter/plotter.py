# %%
from __future__ import annotations
from itertools import repeat
from pathlib import Path
from typing import Optional
import dateutil
from matplotlib.axes import Axes
import matplotlib.pyplot as plt
import numpy as np
from tqdm import tqdm
import xarray as xr
from datetime import datetime, timezone
import logging
import struct
from crc import Calculator, Crc16

crc16xmodem = Calculator(Crc16.XMODEM)  # type: ignore

# %%
SINGLE_MEASUREMENT_SIZE = 20  # bytes
START_PATTERN = b''.join([b'CHARIOT\n']*(256//8))
END_PATTERN = b''.join([b'\xff'] * SINGLE_MEASUREMENT_SIZE)

ACCEL_CODE = 0xACC1
GYRO_CODE = 0x6E50
MAG_CODE = 0x9A61
TEMP_CODE = 0x7E70
BARO_CODE = 0xB480
# %%
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# %%


def process_chunk(chunk: bytes) -> Optional[xr.Dataset]:
    if len(chunk) < SINGLE_MEASUREMENT_SIZE:
        logger.warning("Chunk is too small to contain any measurements")
        return None
    idx = 0
    accel = [[], []]
    gyro = [[], []]
    mag = [[], []]
    temp = {}
    pressure = [[], []]
    altitude = [[], []]
    num_measurements = 0
    pbar = tqdm(total=len(chunk))
    idx = 0
    while len(chunk) > 0:
        measurement = chunk[:SINGLE_MEASUREMENT_SIZE]
        chunk = chunk[SINGLE_MEASUREMENT_SIZE:]
        if measurement == END_PATTERN:
            idx += SINGLE_MEASUREMENT_SIZE
            pbar.update(SINGLE_MEASUREMENT_SIZE)
            continue
        if len(measurement) != SINGLE_MEASUREMENT_SIZE:
            idx += SINGLE_MEASUREMENT_SIZE
            pbar.update(SINGLE_MEASUREMENT_SIZE)
            logger.warning(
                f"Skipping incomplete measurement at index {idx}")
            continue
        mtype = struct.unpack('<H', measurement[0:2])[0]
        data = measurement[2:14]
        tstamp, crc = struct.unpack('<IH', measurement[14:20])
        # Validate CRC
        calc_crc = crc16xmodem.checksum(measurement[:-2])
        if calc_crc != crc:
            logger.warning(
                f"Skipping measurement with invalid CRC at index {idx} (expected {crc:#04x}, got {calc_crc:#04x})")
        else:
            # if True:
            # process data
            if mtype == ACCEL_CODE:
                x, y, z = struct.unpack('<fff', bytes(data[:12]))
                accel[0].append(tstamp)
                accel[1].append((x, y, z))
                num_measurements += 1
            elif mtype == GYRO_CODE:
                x, y, z = struct.unpack('<fff', bytes(data[:12]))
                gyro[0].append(tstamp)
                gyro[1].append((x, y, z))
                num_measurements += 1
            elif mtype == MAG_CODE:
                x, y, z = struct.unpack('<fff', bytes(data[:12]))
                mag[0].append(tstamp)
                mag[1].append((x, y, z))
                num_measurements += 1
            elif mtype == TEMP_CODE:
                source, temperature = struct.unpack('<8sf', bytes(data[:12]))
                source = source.decode(
                    'utf-8', errors='SurrogateEscape').rstrip('\x00')
                if source not in temp:
                    temp[source] = []
                temp[source].append((tstamp, temperature))
                num_measurements += 1
            elif mtype == BARO_CODE:
                temperature, pres, alt = struct.unpack(
                    '<fff', bytes(data[:12]))
                pressure[0].append(tstamp)
                pressure[1].append(pres)
                altitude[0].append(tstamp)
                altitude[1].append(alt)
                if 'barometer' not in temp:
                    temp['barometer'] = []
                temp['barometer'].append((tstamp, temperature))
                num_measurements += 1
            else:
                logger.warning(
                    f"Skipping measurement with unknown type {mtype:#04x} at index {idx}")
        idx += SINGLE_MEASUREMENT_SIZE
        pbar.update(SINGLE_MEASUREMENT_SIZE)
    if num_measurements == 0:
        logger.warning("No valid measurements found in chunk")
        return None
    pbar.close()
    accel_data_array = xr.DataArray(
        np.array(accel[1], dtype=np.float32),
        coords={"time": accel[0], "axes": ['x', 'y', 'z']},
        dims=["time", "axes"],
        name="acceleration",
        attrs={"units": "g"}
    )
    gyro_data_array = xr.DataArray(
        np.array(gyro[1], dtype=np.float32),
        coords={"time": gyro[0], "axes": ['x', 'y', 'z']},
        dims=["time", "axes"],
        name="gyroscope",
        attrs={"units": "dps"}
    )
    mag_data_array = xr.DataArray(
        np.array(mag[1], dtype=np.float32),
        coords={"time": mag[0], "axes": ['x', 'y', 'z']},
        dims=["time", "axes"],
        name="magnetometer",
        attrs={"units": "mG"}
    )
    pressure_data_array = xr.DataArray(
        np.array(pressure[1], dtype=np.float32),
        coords={"time": pressure[0]},
        dims=["time"],
        name="pressure",
        attrs={"units": "mbar"}
    )
    altitude_data_array = xr.DataArray(
        np.array(altitude[1], dtype=np.float32),
        coords={"time": altitude[0]},
        dims=["time"],
        name="altitude",
        attrs={"units": "feet"}
    )
    temp_data_arrays = dict()
    for source, readings in temp.items():
        temp_data_arrays[source] = xr.DataArray(
            np.array([t[1] for t in readings], dtype=np.float32),
            coords={"time": [t[0] for t in readings]},
            dims=["time"],
            attrs={"units": "degC"},
        )
    temp_dataset = xr.concat(
        temp_data_arrays.values(), dim="source")
    temp_dataset = temp_dataset.assign_coords(
        source=list(temp_data_arrays.keys()))
    ds = xr.Dataset({
        'accel': accel_data_array,
        'gyro': gyro_data_array,
        'mag': mag_data_array,
        'pressure': pressure_data_array,
        'altitude': altitude_data_array,
        'temp': temp_dataset,
    })
    return ds

# %%


def process_binfile(binfile: Path, flight_tstamp: Optional[datetime] = None, postflight_tstamp: Optional[datetime] = None) -> list[xr.Dataset]:
    with binfile.open("rb") as f:
        data = f.read()
    chunks = []
    while len(data) > 0:
        start_idx = data.find(START_PATTERN)
        if start_idx == -1:
            break
        data = data[start_idx+len(START_PATTERN):]
        next_idx = data.find(START_PATTERN)
        if next_idx == -1:
            chunk = data[:]
        else:
            chunk = data[:next_idx]
        chunks.append(chunk)
        if next_idx == -1:
            break
    if len(chunks) == 0:
        raise ValueError(f"No valid data chunks found in {binfile}")
    elif len(chunks) > 2:
        logger.warning(
            f'Found {len(chunks)} data chunks, did the system reboot during measurement?')
    datasets = list(map(process_chunk, chunks))
    datasets = list(filter(None, datasets))
    if len(datasets) < 1:
        raise ValueError(f"No valid data found in {binfile}")
    attr = {
        'source': str(binfile.name),
        'processed_at': datetime.now(tz=timezone.utc).isoformat(),
    }
    if flight_tstamp is not None:
        attr['start'] = flight_tstamp.isoformat()
    datasets[0].attrs.update(attr)
    if postflight_tstamp is not None and len(datasets) > 1:
        datasets[1].attrs['start'] = postflight_tstamp.isoformat()
    return datasets
# %%


def xyz_to_rtp(ds: xr.DataArray, axlabel: str = 'axes', tlabel: str = 'time') -> xr.DataArray:
    """Convert x,y,z data to r, theta, phi (spherical coordinates)"""
    x = ds.sel({axlabel: 'x'})
    y = ds.sel({axlabel: 'y'})
    z = ds.sel({axlabel: 'z'})
    r = np.sqrt(x**2 + y**2 + z**2)
    theta = np.acos(z / r) * 180 / np.pi  # polar angle
    phi = np.arctan2(y, x) * 180 / np.pi  # azimuthal angle
    rtp = np.stack([r, theta, phi], axis=-1)
    ods = xr.DataArray(
        np.array(rtp, dtype=np.float32),
        coords={tlabel: ds[tlabel], axlabel: ['r', 'theta', 'phi']},
        dims=[tlabel, axlabel],
    )
    ods.attrs.update(ds.attrs)
    return ods


def plot_datasets(datasets: list[xr.Dataset]):
    # outputdir.mkdir(parents=True, exist_ok=True)
    for i, ds in enumerate(datasets):
        fig, axes = plt.subplots(5, 1, figsize=(6.4, 6), sharex=True, dpi=300)
        fig.subplots_adjust(right=0.75)
        if i == 0:
            fig.suptitle(f"Flight Data ({ds.attrs.get('start', 'unknown start time')})")
            start = dateutil.parser.isoparse(ds.attrs.get('start', '1970-01-01T00:00:00+0000'))
        elif i == 1:
            fig.suptitle(f"Postflight Data ({ds.attrs.get('start', 'unknown start time')})")
            start = dateutil.parser.isoparse(ds.attrs.get('start', '1970-01-01T00:00:00+0000'))
        else:
            plt.close(fig)
            break
        times = np.asarray([start + np.timedelta64(t, 'ms') for t in ds.time.values])
        # Accel
        lines = []
        tay = axes[0].twinx()
        data = xyz_to_rtp(ds['accel'])
        for ax, axis, color in zip([axes[0], tay, tay], ['r', 'theta', 'phi'], ['k', 'r', 'b']):
            ax: Axes = ax
            dr = data.sel(axes=axis)
            mask = ~np.isnan(dr.values)
            l ,= ax.plot(times[mask], dr.values[mask], color=color, label=axis)
            lines.append(l)
        axes[0].set_ylabel("Acceleration (g)")
        tay.set_ylabel("Angle (deg)")
        axes[0].legend(lines, ['|a|', 'θ', 'φ'], loc='upper right')
        # Gyro
        lines = []
        # tay = axes[1].twinx()
        data =ds['gyro']
        for ax, axis, color in zip(repeat(axes[1]), ['x', 'y', 'z'], ['k', 'r', 'b']):
            ax: Axes = ax
            dr = data.sel(axes=axis)
            mask = ~np.isnan(dr.values)
            l, = ax.plot(times[mask], dr.values[mask], color=color, label=axis)
            lines.append(l)
        axes[1].set_ylabel("Angular Velocity (dps)")
        tay.set_ylabel("Angle (deg)")
        axes[1].legend(lines, ['$ω_x$', '$ω_y$', '$ω_z$'], loc='upper right')
        # Magnetometer
        lines = []
        tay = axes[2].twinx()
        data = xyz_to_rtp(ds['mag'])
        for ax, axis, color in zip([axes[2], tay, tay], ['r', 'theta', 'phi'], ['k', 'r', 'b']):
            ax: Axes = ax
            dr = data.sel(axes=axis)
            mask = ~np.isnan(dr.values)
            l, = ax.plot(times[mask], dr.values[mask], color=color, label=axis)
            lines.append(l)
        axes[2].set_ylabel("Magnetic Field (mG)")
        tay.set_ylabel("Angle (deg)")
        axes[2].legend(lines, ['|B|', 'θ', 'φ'], loc='upper right')
        # Temperature
        lines = []
        for source in ds['temp'].source.values:
            dr = ds['temp'].sel(source=source)
            mask = ~np.isnan(dr.values)
            l, = axes[3].plot(times[mask], dr.values[mask], label=str(source).capitalize())
            lines.append(l)
        axes[3].set_ylabel("Temperature (°C)")
        axes[3].legend(lines, [l.get_label() for l in lines], loc='upper right')
        # Pressure and Altitude
        lines = []
        tay = axes[4].twinx()
        for ax, data, label, color in zip([axes[4], tay], [ds['pressure'], ds['altitude']], ["Pressure (mbar)", "Altitude (feet)"], ['k', 'b']):
            ax: Axes = ax
            dr = data
            mask = ~np.isnan(dr.values)
            l, = ax.plot(times[mask], dr.values[mask], color=color, label=label)
            lines.append(l)
        axes[4].set_ylabel("Pressure (mbar)")
        tay.set_ylabel("Altitude (feet)")
        axes[4].legend(lines, ["Pressure", "Altitude"], loc='upper right')
        axes[4].set_xlabel("Time (UTC)")
        axes[-1].set_xticklabels(axes[-1].get_xticklabels(), rotation=45, ha='left')
        fig.show()


# %%
dss = process_binfile(Path('../flashdump/flashdump_20250923-175849.bin'))
# %%
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("binfile", type=Path,
                        help="Path to the flash dump binary file")
    parser.add_argument(
        "postflight_log", type=Path,
        help="Path to the postflight log file",
        nargs='?',
    )
    parser.add_argument(
        "--outputdir", "-o",
        type=Path,
        default=Path("plots"),
        help="Output directory (default: plots)"
    )
    args = parser.parse_args()
    binfile: Path = args.binfile
    if not binfile.exists():
        raise FileNotFoundError(f"Binary file {binfile} does not exist")
    # Get timestamp from filename
    import re
    time_str = re.search(r'([0-9]{8}-[0-9]{6})', binfile.stem)
    if not time_str:
        raise ValueError(
            f"Binary file name {binfile} does not contain a valid timestamp: {time_str}")
    time_str = time_str.group(0)
    flight_tstamp = datetime.strptime(time_str + '+0000', '%Y%m%d-%H%M%S%z')
    if args.postflight_log:
        time_str = re.search(
            r'([0-9]{8}-[0-9]{6})', args.postflight_log.stem)
        if not time_str:
            raise ValueError(
                f"Postflight log file name {args.postflight_log} does not contain a valid timestamp: {time_str}")
        time_str = time_str.group(0)
        postflight_tstamp = datetime.strptime(
            time_str + '+0000', '%Y%m%d-%H%M%S%z')
    else:
        postflight_tstamp = None
