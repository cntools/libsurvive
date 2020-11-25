import pysurvive
import numpy as np
import matplotlib.pyplot as plt
from functools import partial
import math
import bisect
from collections import defaultdict
import scipy
import scipy.spatial.transform

def insert_blanks(v, time_s=.1):
    rtn = []
    last_time = None
    for row in np.transpose(v):
        t = row[0]
        val = row[1]
        if last_time is not None and last_time + time_s < t:
            rtn.append(((last_time + time_s) / 2., None))
            rtn.append(((t - time_s) / 2., None))
        rtn.append((t, val))
        last_time = t
    return np.array(rtn)


def plot_with_time(ax, data, data_fn=id, **kwargs):
    ax.plot([x[0] for x in data],
            [data_fn(x[1]) for x in data], **kwargs)


def get_times(d):
    return [x[0] for x in d]


def get_data(d):
    return np.array([x[1] for x in d])


def figsize_or_default(figsize):
    if figsize is None:
        return (16, 8)
    return figsize


def kabsch_tx(a, b):
    a_center = np.mean(a, axis=0)
    b_center = np.mean(b, axis=0)
    r = scipy.spatial.transform.Rotation.match_vectors(a - a_center, b - b_center)[0]
    return r.apply(b - b_center) + a_center

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
        self.poses = []
        self.velocities = []
        self.raw_angles = defaultdict(list)
        self.datalogs = defaultdict(list)

    def record_imu(self, time, mode, accelgyro, timecode, id):
        self.imu_times.append(time)
        self.gyros.append(accelgyro[3:6])
        self.accels.append(accelgyro[0:3])
        self.time_since_move.append(pysurvive.SurviveSensorActivations_stationary_time(self.so.contents.activations) /
                                    48000000.)

    def record_pose(self, time, timecode, pose):
        self.poses.append((time, pose))

    def record_velocity(self, time, timecode, velocity):
        self.velocities.append((time, velocity))

    def record_light(self, time, sensor_id, acode, timeinsweep, timecode, length, lh):
        key = (lh, acode & 1)
        if sensor_id < 0:
            self.angle_per_sweep[key].append((time, []))

    def record_sweep_angle(self, time, lh, sensor_id, timecode, plane, angle):
        key = (sensor_id, lh, plane)
        all_sensors_key = (lh, plane)
        if len(self.angle_per_sweep[all_sensors_key]):
            self.angle_per_sweep[all_sensors_key][-1][1].append(angle)
        self.angles[key].append([time, angle])

    def record_datalog(self, time, name, values):
        self.datalogs[str(name)].append([time, values])

    def record_angle(self, time, sensor_id, acode, timecode, length, angle, lh):
        key = (sensor_id, lh, acode & 1)
        self.record_sweep_angle(time, lh, sensor_id, timecode, acode & 1, angle)
        self.lengths[key].append([time, length])

    def record_sync(self, time, channel, timeinsweep, ootx, gen):
        self.angle_per_sweep[(channel, 0)].append((time, []))
        self.angle_per_sweep[(channel, 1)].append((time + 0.0166666667, []))

    def record_sweep(self, time, channel, sensor_id, timecode, flag):
        bsd_idx = pysurvive.get_bsd_idx(self.so.contents.ctx, channel)
        if bsd_idx == -1:
            return
        last_sweep = self.so.contents.last_sync_time[bsd_idx]
        time_since_sync = (pysurvive.timecode_difference(timecode, last_sweep) / 48000000.)

        hz = 48000000. / last_sweep if last_sweep > 0 else 1.
        time_per_rot = 1. / hz

        if time_since_sync > time_per_rot:
            return

        key = (sensor_id, channel)
        raw_angle = time_since_sync / time_per_rot * 2. * math.pi
        self.raw_angles[key].append([time, raw_angle])

        # int8_t bsd_idx = survive_get_bsd_idx(ctx, channel);
        # if (bsd_idx == -1) {
        # SV_WARN("Invalid channel requested(%d) for %s", channel, so->codename)
        # return;
        # }
        #
        # survive_notify_gen2(so, "sweep called");
        #
        # if (ctx->calptr) {
        # // survive_cal_light( so, sensor_id, acode, timeinsweep, timecode, length, lh);
        # }
        #
        # survive_recording_sweep_process(so, channel, sensor_id, timecode, half_clock_flag);
        #
        # survive_timecode last_sweep = so->last_sync_time[bsd_idx];
        # assert(channel <= NUM_GEN2_LIGHTHOUSES);
        #
        # // SV_INFO("Sensor ch%2d %2d %d %12x %6d", channel, sensor_id, flag, timecode, timecode
        # // so->last_sync_time[bsd_idx]);
        #
        # FLT time_since_sync = (survive_timecode_difference(timecode, last_sweep) / 48000000.);
        # // if (half_clock_flag)
        # //	time_since_sync += 0.5 / 48000000.;
        #
        # FLT hz = 48000000. / so->last_time_between_sync[bsd_idx];
        # FLT time_per_rot = 1. / hz;
        #
        # if (time_since_sync > time_per_rot)
        # return;
        #
        # FLT angle = time_since_sync / time_per_rot * 2. * LINMATHPI;

        pass

    # SURVIVE_EXPORT void survive_default_sync_process(SurviveObject *so, survive_channel channel,
    #                                                                                    survive_timecode timeinsweep, bool ootx, bool gen);
    # SURVIVE_EXPORT void survive_default_sweep_process(SurviveObject *so, survive_channel channel, int sensor_id,
    #                                                                                                  survive_timecode timecode, bool flag);
    # SurviveObject *so, survive_channel channel, int sensor_id,
    # survive_timecode timecode, int8_t plane, FLT angle
    def plot_accel(self, fig=None, plot_num=1, plot_rows=1, plot_cols=1, only_avg=True, figsize=None, **kwargs):
        if fig is None:
            fig = plt.figure(figsize=figsize_or_default(figsize))

        accel_running_avg = self.datalogs['accel running average']
        if len(accel_running_avg) == 0:
            only_avg = False

        moveThreshAcc = pysurvive.configf(self.so.contents.ctx, "move-threshold-acc", pysurvive.SC_GET, 0)
        ax = fig.add_subplot(plot_rows, plot_cols, plot_num, title=self.name + ' Accels')

        ax.plot([self.imu_times[0], self.imu_times[-1]], [moveThreshAcc] * 2, '--', linewidth=1.5,
                label="Move threshold")

        if not only_avg:
            ax.plot(self.imu_times[1:], np.linalg.norm(np.diff(self.accels, axis=0), axis=1), linewidth=0.5,
                    label="Norm diff")
            ax.plot(self.imu_times, np.linalg.norm(self.accels, axis=1), linewidth=1, label="Norm")

        accel_running_avg = self.datalogs['accel running average']
        if len(accel_running_avg):
            plot_with_time(ax, accel_running_avg, data_fn=np.linalg.norm, label="Running accel average")
            start = len(self.imu_times) - len(get_data(accel_running_avg))
            ax.plot(self.imu_times[start:], np.linalg.norm(self.accels[start:] - get_data(accel_running_avg), axis=1),
                    linewidth=0.5, label="Norm diff(avg)")
            ax.plot(get_times(accel_running_avg)[1:],
                    np.linalg.norm(np.diff(get_data(accel_running_avg), axis=0), axis=1),
                    label="Running accel average diff")

        accels = np.array(self.accels)
        axes_name = ['X', 'Y', 'Z']
        for i in range(3):
            if not only_avg:
                ax.plot(self.imu_times, accels[:, i], linewidth=0.5, label=axes_name[i])
            ax.plot(get_times(accel_running_avg), get_data(accel_running_avg)[:, i], label="Avg" + axes_name[i])
        ax.legend()

    def plot_datalog(self, prefix="", fig=None, plot_num=1, plot_rows=1, plot_cols=1, figsize=None, *args, **kwargs):
        if fig is None:
            fig = plt.figure(figsize=figsize_or_default(figsize))
        ax = fig.add_subplot(plot_rows, plot_cols, plot_num, title=prefix)

        keys = [k for k in self.datalogs.keys() if k.startswith(prefix)]
        for k in keys:
            times = get_times(self.datalogs[k])
            data = get_data(self.datalogs[k])

            ax.plot(times, data, '-', linewidth=1, ms=1, label=k)
        ax.legend()
        fig.tight_layout()

    def plot_gyro(self, fig=None, plot_num=1, plot_rows=1, plot_cols=1, figsize=None, *args, **kwargs):
        if fig is None:
            fig = plt.figure(figsize=figsize_or_default(figsize))
        ax = fig.add_subplot(plot_rows, plot_cols, plot_num, title=self.name + ' Gyros')

        moveThreshGyro = pysurvive.configf(self.so.contents.ctx, "move-threshold-gyro", pysurvive.SC_GET, 0)

        axes_name = ['X', 'Y', 'Z']
        gyros = np.array(self.gyros)
        ax.plot([self.imu_times[0], self.imu_times[-1]], [moveThreshGyro] * 2, linewidth=1)
        ax.plot(self.imu_times, np.linalg.norm(gyros, axis=1), linewidth=1, label='Norm')
        for i in range(3):
            ax.plot(self.imu_times, gyros[:, i], linewidth=1, label=axes_name[i])
        ax.legend()

    def plot_imu(self, fig=None, plot_num=1, plot_rows=1, plot_cols=2, figsize=None, *args, **kwargs):
        if fig is None:
            fig = plt.figure(figsize=figsize_or_default(figsize))

        self.plot_gyro(fig, plot_num=plot_num, *args, **kwargs)
        self.plot_accel(fig, plot_num=plot_num + 1, *args, **kwargs)

        return 2

    def plot_pose_diff(self, fig=None, plot_num=1, plot_rows=1, plot_cols=1, figsize=None, **kwargs):
        if len(self.poses) == 0:
            return 0

        if fig is None:
            fig = plt.figure()

        ax = fig.add_subplot(plot_rows, plot_cols, plot_num, title=self.name + ' pose diff')

        times = [x[0] for x in self.poses]
        poses = [x[1][0:3] for x in self.poses]

        print(np.diff(times))
        print(np.diff(poses, axis=0))

        data = np.stack([times[1:], np.linalg.norm(
            np.diff(poses, axis=0) / np.diff(times)[:, None], axis=1
        )])

        diff_data = insert_blanks(data)
        times = diff_data[:, 0]
        data = diff_data[:, 1]
        ax.plot(times, data, linewidth=1)
        return 1

    def plot_light_diff(self, fig=None, plot_num=1, plot_rows=1, plot_cols=1, figsize=None, norm_diff=True, **kwargs):
        if len(self.angles) == 0:
            return 0

        if fig is None:
            fig = plt.figure(figsize=figsize_or_default(figsize))

        ax = fig.add_subplot(plot_rows, plot_cols, plot_num, title=self.name + ' light diff')
        moveThreshAng = pysurvive.configf(self.so.contents.ctx, "move-threshold-ang", pysurvive.SC_GET, 0)
        ax.plot([self.imu_times[0], self.imu_times[-1]], [moveThreshAng] * 2, linewidth=1)

        for k, v in self.angles.items():
            if len(v) <= 1:
                continue
            vv = np.array(v)
            norm = np.diff(vv[:, 0]) if norm_diff else 1.
            data = np.stack([vv[1:, 0], np.array(np.diff(vv[:, 1]) / norm)])
            diff_data = insert_blanks(data)
            times = diff_data[:, 0]
            data = diff_data[:, 1]
            ax.plot(times, data, '-', label=k, linewidth=1)
        return 1

    def plot_moving(self, fig=None, plot_num=1, plot_rows=1, plot_cols=1, move_tolerance=.5, figsize=None, **kwargs):
        if fig is None:
            fig = plt.figure(figsize=figsize_or_default(figsize))

        ax = fig.add_subplot(plot_rows, plot_cols, plot_num, title=self.name + ' not moving')

        vv = np.array(self.time_since_move)
        ax.plot(self.imu_times, vv.clip(0, 1.0), linewidth=1, label='Time since move')
        ax.plot(self.imu_times, vv.clip(0, 1.0) >= move_tolerance, linewidth=1, label='Not moving cutoff')
        ax.legend()
        return 1

    def plot_light_angles(self, fig=None, plot_num=1, plot_rows=1, plot_cols=1, figsize=None, plot_all_light=False,
                          **kwargs):
        if len(self.angles) == 0:
            return 0

        if fig is None:
            fig = plt.figure()

        def pv(tv):
            time, values = tv
            if len(values) == 0:
                return None
            return (time, np.mean(values))

        ax = fig.add_subplot(plot_rows, plot_cols, plot_num, title=self.name + ' light')
        # for k,v in self.angle_per_sweep.items():
        #     vv = np.array(list(filter(lambda x: x is not None, map(pv, v))))
        #     if len(vv):
        #         ax.plot(vv[:, 0], (vv[:, 1]), label=k, linewidth=1)

        for k, v in self.angles.items():
            vv = np.array(v)
            if vv.shape[0] > 10:
                ax.plot(vv[:, 0], vv[:, 1], '.', label=k, linewidth=1)
        # ax.legend(ncol=4)

        return 1

    def plot_against_gt(self, gt, fig=None, plot_num=1, plot_rows=1, plot_cols=1, figsize=None, **kwargs):
        if fig is None:
            fig = plt.figure(figsize=figsize_or_default(figsize))
        stands = self.standstills()
        mean_positions = (np.array([np.mean([y[1] for y in x[1]], axis=0)[0:3] for x in stands]))[1:]
        center = np.mean(mean_positions, axis=0)
        gt_center = np.mean(gt, axis=0)
        r = scipy.spatial.transform.Rotation.match_vectors(mean_positions - center, gt - gt_center)[0]
        tx_gt = r.apply(gt - gt_center) + center

        ax = fig.add_subplot(plot_rows, plot_cols, plot_num, title='Positional Error')
        ax.set_xlabel("Time(s)")
        ax.set_ylabel("mm")
        for i in range(len(tx_gt)):
            times = [y[0] for y in stands[i+1][1]]
            dd = np.linalg.norm([y[1][0:3] for y in stands[i+1][1][:]] - tx_gt[i],axis=1)
            ax.plot(times, dd*1000)
        fig.tight_layout()
        return ax

    def plot_light(self, fig=None, plot_num=1, plot_rows=3, plot_cols=1, figsize=None, **kwargs):
        if len(self.angles) == 0:
            return 0

        graphs = 0
        if fig is None:
            fig = plt.figure(figsize=figsize_or_default(figsize))

        if len(self.lengths):
            ax = fig.add_subplot(plot_rows, plot_cols, plot_num, title=self.name + ' Light lengths')
            for k, v in self.lengths.items():
                times = [x[0] for x in v]
                v = np.array([x[1] for x in v])
                if v.shape[0] > 0:
                    ax.plot(times, v, linewidth=1, label=k)
            graphs += 1

        graphs += self.plot_light_angles(fig, plot_num + graphs, plot_rows + graphs, plot_cols, figsize, **kwargs)

        graphs += self.plot_light_diff(fig, plot_num=plot_num + graphs, plot_cols=plot_cols, plot_rows=plot_rows,
                                       **kwargs)
        #fig.tight_layout()

        return graphs

    def plot(self, fig=None, plot_num=1, figsize=None, plot_rows=3, plot_cols=2, **kwargs):
        if len(self.angles) == 0 or len(self.imu_times) == 0:
            return 0

        if fig is None:
            fig = plt.figure(figsize=figsize)
        plot_num += self.plot_imu(fig=fig, plot_rows=plot_rows, plot_num=plot_num, plot_cols=plot_cols, **kwargs)
        plot_num += self.plot_moving(fig=fig, plot_rows=plot_rows, plot_num=plot_num, plot_cols=plot_cols, **kwargs)
        plot_num += self.plot_light(fig=fig, plot_rows=plot_rows, plot_num=plot_num, plot_cols=plot_cols, **kwargs)
        plot_num += self.plot_pose_diff(fig=fig, plot_rows=plot_rows, plot_num=plot_num, plot_cols=plot_cols, **kwargs)
        fig.tight_layout()
        return plot_num - 1

    def standstill_times(self):
        idxs = []
        last_val = self.time_since_move[0]
        for idx, val in enumerate(self.time_since_move):
            if last_val > val and last_val > .5:
                idxs.append(idx - 1)
            last_val = val
        if last_val > .5:
            idxs.append(len(self.time_since_move) - 1)

        return [(self.imu_times[i] - self.time_since_move[i], self.imu_times[i - 1]) for i in idxs]

    def standstills(self):
        ss_times = self.standstill_times()

        ss = []
        for (start_time, end_time) in ss_times:
            start_idx = bisect.bisect_right([x[0] for x in self.poses], start_time)
            end_idx = bisect.bisect_left([x[0] for x in self.poses], end_time)
            if end_idx > start_idx:
                ss.append(((start_time, end_time), self.poses[start_idx:end_idx]))
        return ss


class Recorder:
    def __init__(self):
        self.ctx = None
        self.data = {}

    def get(self, so):
        codename = so.contents.codename.decode('utf8')
        if codename not in self.data:
            self.data[codename] = RecordedData(so)
        return self.data[codename]

    def plot(self, fig=None, figsize=None, **kwargs):
        plot_num = 1
        plot_rows = len(self.data.items()) * 8 // 2

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

    def cb_fn(class_fn, so, *args):
        dat = recorder.get(so)
        time = pysurvive.survive_run_time(so.contents.ctx)
        return class_fn(dat, time, *args)

    pysurvive.install_angle_fn(ctx, partial(cb_fn, RecordedData.record_angle))
    pysurvive.install_light_fn(ctx, partial(cb_fn, RecordedData.record_light))
    pysurvive.install_imu_fn(ctx, partial(cb_fn, RecordedData.record_imu))
    pysurvive.install_pose_fn(ctx, partial(cb_fn, RecordedData.record_pose))
    pysurvive.install_velocity_fn(ctx, partial(cb_fn, RecordedData.record_velocity))
    pysurvive.install_sweep_fn(ctx, partial(cb_fn, RecordedData.record_sweep))
    pysurvive.install_sync_fn(ctx, partial(cb_fn, RecordedData.record_sync))
    pysurvive.install_sweep_angle_fn(ctx, partial(cb_fn, RecordedData.record_sweep_angle))
    pysurvive.install_datalog_fn(ctx, partial(cb_fn, RecordedData.record_datalog))
    return recorder
