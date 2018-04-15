#!/usr/bin/env python
import os
import shutil
import rospy
import rosbag
import datetime
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import speed_debug_msgs.msg
from mpl_toolkits.mplot3d import axes3d


def general_twoD_plot(y, x=None, title=None, xlabel=None, ylabel=None, startlabel=None, endlabel=None,
                      workdir=None, show=False, save=True, format='svg'):
    points_num = len(y)
    if not bool(points_num):
        print 'point number is zero'
        return
    if workdir:
        os.chdir(workdir)
    if title:
        figure = title
    else:
        figure = 'title'
    fig = plt.figure(figure)
    sub_fig_1 = plt.subplot(111)
    if title:
        sub_fig_1.set_title(title)
    if xlabel:
        sub_fig_1.set_xlabel(xlabel)
    if ylabel:
        sub_fig_1.set_ylabel(ylabel)
    if x is not None:
        sub_fig_1.plot(x, y)
        if startlabel:
            sub_fig_1.text(x[0], y[0], startlabel)
        if endlabel:
            sub_fig_1.text(x[points_num - 1], y[points_num - 1], endlabel)
    else:
        sub_fig_1.plot(y)
        if startlabel:
            sub_fig_1.text(0, y[0], startlabel)
        if endlabel:
            sub_fig_1.text(points_num - 1, y[points_num - 1], endlabel)
    if save:
        fig.savefig(figure, format=format)
    if show:
        plt.show(fig)
    fig.clf()
    return


def general_threeD_plot(x, y, z, title=None, xlabel=None, ylabel=None, zlabel=None, startlabel=None, endlabel=None,
                        workdir=None, show=True, save=False, format='svg'):
    points_num = len(x)
    if not bool(points_num):
        print 'point number is zero'
        return
    if workdir:
        os.chdir(workdir)
    if title:
        figure = title
    else:
        figure = 'title'
    fig = plt.figure(figure)
    fig3 = fig.gca(projection='3d')
    if title:
        fig3.set_title(title)
    if xlabel:
        fig3.set_xlabel(xlabel)
    if ylabel:
        fig3.set_ylabel(ylabel)
    if zlabel:
        fig3.set_ylabel(zlabel)
    fig3.plot(x, y, z)
    if startlabel:
        fig3.text(x[0], y[0], z[0], startlabel)
    if endlabel:
        fig3.text(x[points_num - 1], y[points_num - 1], z[points_num - 1], endlabel)
    if save:
        fig.savefig(figure, format=format)
    if show:
        plt.show(fig)
    fig.clf()
    return


def get_arclength(x, y):
    s = []
    s.append(0)
    tmp1 = 0
    num = len(x) - 1
    i = 0
    while i < num:
        tmp1 += np.sqrt(np.power((x[i + 1] - x[i]), 2.0) + np.power((y[i + 1] - y[i]), 2.0))
        s.append(tmp1)
        i += 1
        pass
    return s


def constrained_speed_plot(s, v_max, v_limit_lat_acc, v_limit_lon_acc, v_limit_lon_adc, v_limit_lon_jerk):
    fig = plt.figure('Constrained Speed')
    sub_fig_1 = plt.subplot(111)
    sub_fig_1.set_title('speed with different constraints')
    sub_fig_1.set_xlabel('Arc-length')
    sub_fig_1.set_ylabel('speed')
    sub_fig_1.plot(s, v_max, color='red', label='speed without constraints')
    sub_fig_1.plot(s, v_limit_lat_acc, color='orange', label='latitude acceleration constrained')
    sub_fig_1.plot(s, v_limit_lon_acc, color='yellow', label='longitude acceleration constrained')
    sub_fig_1.plot(s, v_limit_lon_adc, color='green', label='longitude deceleration constrained')
    sub_fig_1.plot(s, v_limit_lon_jerk, color='blue', label='longitude jerk constrained')
    sub_fig_1.legend()
    fig.savefig('constrained_speed', format='svg')
    fig.clf()
    return


def path_plot(points_x, points_y):
    points_num = len(points_x)
    fig = plt.figure('Path')
    sub_fig_1 = plt.subplot(111)
    sub_fig_1.set_title('Path')
    sub_fig_1.set_xlabel('latitude')
    sub_fig_1.set_ylabel('longitude')
    sub_fig_1.plot(points_x, points_y)
    sub_fig_1.text(points_x[0], points_y[0], 'start')
    sub_fig_1.text(points_x[points_num - 1], points_y[points_num - 1], 'end')
    fig.savefig('path', format='svg')
    fig.clf()
    return


def curvature_plot(s, curvature):
    fig = plt.figure('curvature')
    sub_fig_1 = plt.subplot(111)
    sub_fig_1.set_title('Curvature along the Path')
    sub_fig_1.set_xlabel('s')
    sub_fig_1.set_ylabel('curvature')
    sub_fig_1.plot(s, curvature)
    fig.savefig('curvature', format='svg')
    fig.clf()
    return


def msg_analysis(msg, analysis_path, msg_seq, ir_counter, duration_time, cur_vel, cur_vel_actual, cur_latitude, cur_longitude, max_curvature):
    os.chdir(analysis_path)
    os.mkdir(str(msg_seq))
    msg_path = os.path.join(analysis_path, str(msg_seq))
    os.chdir(msg_path)

    x = []
    y = []
    curvature = []
    vel_lon_jerk = []

    for point in msg.trajectory.points:
        x.append(point.position.x)
        y.append(point.position.y)
        curvature.append(point.curvature)
        vel_lon_jerk.append(point.velocity.linear.x)
        pass

    max_curvature.append(max(max(curvature), abs(min(curvature))))

    s = msg.s
    vel_max = msg.vel_max
    vel_lat_acc = msg.vel_limit_lat_acc
    vel_lon_acc = msg.vel_limit_lon_acc
    vel_lon_adc = msg.vel_limit_lon_adc
    ir_counter.append(msg.ir_counter)
    duration_time.append(msg.duration_time)
    cur_vel.append(msg.cur_vel)
    cur_vel_actual.append(msg.cur_vel_actual)
    cur_latitude.append(msg.cur_latitude)
    cur_longitude.append(msg.cur_longitude)

    # calculate time of each point
    time = []
    time.append(0)
    point_num = len(msg.trajectory.points)
    tmp1 = 1
    tmp_time = 0
    while tmp1 < point_num:
        tmp_time += (s[tmp1] - s[tmp1 - 1]) * 2.0 / (vel_lon_jerk[tmp1] + vel_lon_jerk[tmp1 - 1])
        time.append(tmp_time)
        tmp1 += 1
        pass

    raw_data = {'x': x,
                'y': y,
                's': s,
                'curvature': curvature,
                'vel_max': vel_max,
                'vel_lat_acc': vel_lat_acc,
                'vel_lon_acc': vel_lon_acc,
                'vel_lon_adc': vel_lon_adc,
                'vel_lon_jerk': vel_lon_jerk,
                'time': time}

    data_frame = pd.DataFrame(data=raw_data)
    data_frame.to_csv('raw_data.csv')

    general_twoD_plot(y, x, title='path', xlabel='x', ylabel='y', startlabel='start', endlabel='end')
    general_twoD_plot(curvature, s, title='curvature along path', xlabel='s', ylabel='curvature')
    general_twoD_plot(s, time, title='time', xlabel='time', ylabel='s')


    # path_plot(x, y)
    # curvature_plot(s, curvature)
    constrained_speed_plot(s, vel_max, vel_lat_acc, vel_lon_acc, vel_lon_adc, vel_lon_jerk)

    return


def bag_analysis(file, bagfiles_path):
    os.chdir(bagfiles_path)
    file_name = os.path.splitext(file)[0]
    if not os.path.exists(file_name):
        print 'I am handling a bagfile slowly, not stuck, please waiting ...'
        bag = rosbag.Bag(file)
        os.mkdir(file_name)
        analysis_path = os.path.join(bagfiles_path, file_name)
        os.chdir(analysis_path)
        msg_seq = 0
        ir_counter = []
        duration_time = []
        cur_vel = []
        cur_vel_actual = []
        cur_latitude = []
        cur_longitude = []
        max_curvature = []
        for topic, msg, t in bag.read_messages():
            msg_seq += 1
            msg_analysis(msg, analysis_path, msg_seq, ir_counter, duration_time, cur_vel, cur_vel_actual, cur_latitude, cur_longitude, max_curvature)
            pass
        s_global_gps = get_arclength(cur_latitude, cur_longitude)
        addition_data = {'ir_counter': ir_counter,
                         'duration_time': duration_time,
                         'cur_vel': cur_vel,
                         'cur_vel_actual': cur_vel_actual,
                         'cur_latitude': cur_latitude,
                         'cur_longitude': cur_longitude,
                         'max_curvature': max_curvature,
                         's_global_gps': s_global_gps}
        addition_frame = pd.DataFrame(data=addition_data)
        os.chdir(analysis_path)
        addition_frame.to_csv('addition_data.csv')

        general_twoD_plot(ir_counter, title='ir_counter', ylabel='ir_counter')
        general_twoD_plot(duration_time, title='duration_time', ylabel='duration_time')
        general_twoD_plot(cur_vel_actual, s_global_gps, title='vel along gps path', xlabel='s along gps', ylabel='actual velocity')
        general_twoD_plot(max_curvature, s_global_gps, title='max curvature along gps path', xlabel='s along gps', ylabel='max_curvature')
    else:
        print ('the dir \'' + file_name + '\' already exists, do nothing with the bagfile')
        pass
    return


def bag_read():
    origin_path = os.getcwd()
    bagfiles_path = os.path.join(origin_path, 'bagfiles')
    os.chdir(bagfiles_path)
    for root, dirs, files in os.walk(bagfiles_path):
        for file in files:
            if os.path.splitext(file)[1] == '.bag':
                bag_analysis(file, bagfiles_path)
    return


if __name__ == '__main__':
    bag_read()
