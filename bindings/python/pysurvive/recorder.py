import pysurvive
import numpy as np
import matplotlib.pyplot as plt

from collections import defaultdict

def insert_blanks(v, time_s = .1):
    rtn = []
    last_time = None
    for row in np.transpose(v):
        t = row[0]
        val = row[1]
        if last_time is not None and last_time + time_s < t:
            rtn.append( ((last_time + time_s) / 2., None))
            rtn.append( ((t - time_s) / 2., None))
        rtn.append((t, val))
        last_time = t
    return np.array(rtn)

class RecordedData:
    def __init__(self, so):
        self.so = so
        self.name = self.so.contents.codename.decode('utf8')
        self.imu_times = []
        self.gyros = []
        self.accels = []
        self.angles = defaultdict(list)
        self.lengths = defaultdict(list)
        self.angle_per_sweep = defaultdict(list)
        self.time_since_move = []

    def record_imu(self, time, mode, accelgyro, timecode, id):
        self.imu_times.append(time)
        self.gyros.append(accelgyro[3:6])
        self.accels.append(accelgyro[0:3])
        self.time_since_move.append(pysurvive.SurviveSensorActivations_stationary_time(self.so.contents.activations) /
                                    48000000.)

    def record_light(self, time, sensor_id, acode, timeinsweep, timecode, length, lh):
        key = (lh, acode & 1)
        if sensor_id < 0:
            self.angle_per_sweep[key].append((time, []))

    def record_angle(self, time, sensor_id, acode, timecode, length, angle, lh):
        key = (sensor_id, lh, acode & 1)
        all_sensors_key = (lh, acode & 1)
        if len(self.angle_per_sweep[all_sensors_key]):
            self.angle_per_sweep[all_sensors_key][-1][1].append(angle)
        self.angles[key].append([time, angle])
        self.lengths[key].append([time, length])

    def plot_imu(self, fig = None, plot_num = 1, plot_rows = 2, plot_cols = 2, figsize=None, **kwargs):
        if fig is None:
            fig = plt.figure(figsize=figsize)

        ax = fig.add_subplot(plot_rows, plot_cols, plot_num, title=self.name + ' Gyros')

        moveThreshGyro = pysurvive.configf(self.so.contents.ctx, "move-threshold-gyro", pysurvive.SC_GET, 0)
        moveThreshAcc = pysurvive.configf(self.so.contents.ctx, "move-threshold-acc", pysurvive.SC_GET, 0)

        axes_name = ['X', 'Y', 'Z']
        gyros = np.array(self.gyros)
        ax.plot([self.imu_times[0],self.imu_times[-1]], [moveThreshGyro] * 2, linewidth=1)
        ax.plot(self.imu_times, np.linalg.norm(gyros, axis=1), linewidth=1, label='Norm')
        for i in range(3):
            ax.plot(self.imu_times, gyros[:, i], linewidth=1, label=axes_name[i])
        ax.legend()

        ax = fig.add_subplot(plot_rows, plot_cols, plot_num + 1, title=self.name + ' Accels')

        ax.plot([self.imu_times[0],self.imu_times[-1]], [moveThreshAcc] * 2, linewidth=1)
        ax.plot(self.imu_times[1:], np.linalg.norm(np.diff(self.accels, axis=0), axis=1), linewidth=1)

        return 2

    def plot_light_diff(self, fig = None, plot_num = 1, plot_rows = 1, plot_cols = 1, figsize=None, **kwargs):
        if fig is None:
            fig = plt.figure()

        ax = fig.add_subplot(plot_rows, plot_cols, plot_num, title=self.name + ' light diff')
        moveThreshAng = pysurvive.configf(self.so.contents.ctx, "move-threshold-ang", pysurvive.SC_GET, 0)
        ax.plot([self.imu_times[0],self.imu_times[-1]], [moveThreshAng] * 2, linewidth=1)

        for k,v in self.angles.items():
            vv = np.array(v)
            data = np.stack([vv[1:, 0], np.array(np.diff(vv[:, 1]) / np.diff(vv[:, 0]))])
            diff_data = insert_blanks(data)
            times = diff_data[:, 0]
            data = diff_data[:, 1]
            ax.plot(times, data, label=k, linewidth=1)
        return 1

    def plot_moving(self, fig = None, plot_num = 1, plot_rows = 1, plot_cols = 1, figsize=None, **kwargs):
        if fig is None:
            fig = plt.figure()

        ax = fig.add_subplot(plot_rows, plot_cols, plot_num, title=self.name + ' not moving')

        vv = np.array(self.time_since_move)
        ax.plot(self.imu_times, vv.clip(0, 1.0),linewidth=1)
        ax.plot(self.imu_times, vv.clip(0, 1.0) >= 1.0,linewidth=1)
        return 1

    def plot_light(self, fig = None, plot_num = 1, plot_rows = 2, plot_cols = 2, figsize=None, **kwargs):
        if fig is None:
            fig = plt.figure()

        ax = fig.add_subplot(plot_rows, plot_cols, plot_num, title=self.name + ' Light lengths')

        for k,v in self.lengths.items():
            times = [x[0] for x in v]
            v = np.array([x[1] for x in v])
            if v.shape[0] > 0:
                ax.plot(times, v, linewidth=1, label=k)

        def pv(tv):
            time,values=tv
            if len(values) == 0:
                return None
            return (time, np.mean(values))

        ax = fig.add_subplot(plot_rows, plot_cols, plot_num + 1, title=self.name + ' light')
        for k,v in self.angle_per_sweep.items():
            vv = np.array(list(filter(lambda x: x is not None, map(pv, v))))
            ax.plot(vv[:, 0], (vv[:, 1]), label=k, linewidth=1)

        plot_num += self.plot_light_diff(fig, plot_num=plot_num + 2, plot_cols=plot_cols, plot_rows=plot_rows, **kwargs)

        return 3

    def plot(self, fig = None, plot_num = 1, figsize = None, plot_rows = 3, plot_cols = 2, **kwargs):
        if fig is None:
            fig = plt.figure(figsize=figsize)
        plot_num += self.plot_imu(fig = fig, plot_rows=plot_rows, plot_num=plot_num, plot_cols=plot_cols,**kwargs)
        plot_num += self.plot_moving(fig = fig, plot_rows=plot_rows, plot_num=plot_num, plot_cols=plot_cols,**kwargs)
        plot_num += self.plot_light(fig = fig, plot_rows=plot_rows, plot_num=plot_num, plot_cols=plot_cols,**kwargs)
        fig.tight_layout()
        return plot_num - 1

class Recorder:
    ctx = None
    data = {}

    def get(self, so):
        codename = so.contents.codename.decode('utf8')
        if codename not in self.data:
            self.data[codename] = RecordedData(so)
        return self.data[codename]

    def record_imu(self, so, mode, accelgyro, timecode, id):
        dat = self.get(so)
        time = pysurvive.survive_run_time(so.contents.ctx)
        return dat.record_imu(time, mode, accelgyro, timecode, id)

    def record_light(self, so, sensor_id, acode, timeinsweep, timecode, length, lh):
        dat = self.get(so)
        time = pysurvive.survive_run_time(so.contents.ctx)
        return dat.record_light(time, sensor_id, acode, timeinsweep, timecode, length, lh)

    def record_angle(self, so, sensor_id, acode, timecode, length, angle, lh):
        dat = self.get(so)
        time = pysurvive.survive_run_time(so.contents.ctx)
        return dat.record_angle(time, sensor_id, acode, timecode, length, angle, lh)

    def plot(self, fig=None, figsize=None, **kwargs):
        plot_num = 1
        plot_rows = len(self.data.items()) * 6 // 2

        if fig is None:
            fig = plt.figure(figsize=(14, plot_rows * 2))

        for k, d in self.data.items():
            plot_num += d.plot(fig=fig,
                               plot_rows=plot_rows,
                               plot_num=plot_num,
                               plot_cols=2,
                               **kwargs)
        fig.tight_layout()

def install(ctx):
    recorder = Recorder()

    def imu(*args):
        return recorder.record_imu(*args)
    def light(*args):
        return recorder.record_light(*args)
    def angle(*args):
        return recorder.record_angle(*args)

    pysurvive.install_angle_fn(ctx, angle)
    pysurvive.install_light_fn(ctx, light)
    pysurvive.install_imu_fn(ctx, imu)

    return recorder
