from __future__ import annotations

import math
import matplotlib.pyplot as plt
import re

from Colors import Colors

def read_log_file(filename: str) -> list[str]:
    with open(filename, 'r') as f:
        lines = f.readlines()
    return lines

def main() -> None:
    heading_log: list[str] = read_log_file("/home/thomas/.ros/log/seagraves_unmanned_systems_pkg/heading_log.csv")
    command_log: list[str] = read_log_file("/home/thomas/.ros/log/seagraves_unmanned_systems_pkg/command_log.csv")
    pose_log: list[str] = read_log_file("/home/thomas/.ros/log/seagraves_unmanned_systems_pkg/pose_log.csv")

    # create two sub plots
    # one plot is an x y position graph where the color of the line is a funciton of the time
    # the other plot is a time vs. velocity graph

    current_time: list[float] = []
    current_x: list[float] = []
    current_y: list[float] = []
    current_z: list[float] = []
    current_roll: list[float] = []
    current_pitch: list[float] = []
    current_yaw: list[float] = []
    
    command_time: list[float] = []
    command_x: list[float] = []   # forward
    command_yaw: list[float] = []  # z in the angular vector

    start_time: int = None

    # time,position_x,position_y,position_z,roll,pitch,yaw
    for line in pose_log[1:]:
        current_time.append(float(line.split(",")[0]))
        if start_time is None:
            start_time = int(current_time[-1])
            current_time[-1] = 0
        else:
            current_time[-1] -= start_time
        current_x.append(float(line.split(",")[1]))
        current_y.append(float(line.split(",")[2]))
        current_z.append(float(line.split(",")[3]))
        current_roll.append(float(line.split(",")[4]))
        current_pitch.append(float(line.split(",")[5]))
        current_yaw.append(float(line.split(",")[6]))
        
    # time,linear_x,linear_y,linear_z,angular_x,angular_y,angular_z
    for line in command_log[1:]:
        line_time = float(line.split(",")[0])
        if command_time:
            command_time.append(line_time - start_time)
            command_x.append(command_x[-1])
            command_yaw.append(command_yaw[-1])

        command_time.append(float(line.split(",")[0]) - start_time)
        command_x.append(float(line.split(",")[1]))
        command_yaw.append(float(line.split(",")[6]))

    # time,desired_heading,actual_heading

    fig = plt.figure()
    plt.style.use('dark_background')
    plt.set_cmap("Blues")
    position_subplot = plt.subplot2grid((3, 2), (0, 0), colspan=1)
    rotation_subplot = plt.subplot2grid((3, 2), (1, 0), colspan=1)
    command_subplot = plt.subplot2grid((3, 2), (2, 0), colspan=1)
    xy_subplot = plt.subplot2grid((3, 2), (0, 1), rowspan=3)

    twin_position = position_subplot.twinx()
    twin_command = command_subplot.twinx()

    fig.set_size_inches(9, 6)

    fig.set_facecolor(Colors.grey)
    for subplot in [
        position_subplot, 
        twin_position,
        rotation_subplot, 
        command_subplot, 
        twin_command,
        xy_subplot,
    ]:
        subplot.set_facecolor(Colors.grey)
        subplot.title.set_color(Colors.light_grey)
        subplot.xaxis.label.set_color(Colors.light_grey)
        subplot.yaxis.label.set_color(Colors.light_grey)
        subplot.tick_params(colors=Colors.light_grey)
    
    position_subplot.set_title("Position")
    rotation_subplot.set_title("Rotation")
    command_subplot.set_title("Commands")
    xy_subplot.set_title("XY Position")

    # plotting
    position_subplot.plot(current_time, current_x, color=Colors.red, label="x")
    twin_position.plot(current_time, current_y, color=Colors.blue, label="y")

    rotation_subplot.plot(current_time, current_yaw, color=Colors.red, label="yaw")

    command_subplot.plot(command_time, command_x, color=Colors.red, label="x", marker="o")
    twin_command.plot(command_time, command_yaw, color=Colors.blue, label="yaw", marker="o")

    xy_subplot.scatter(current_x, current_y, c=current_time, label="xy")

    min_axis = min(min(current_x), min(current_y))
    max_axis = max(max(current_x), max(current_y))
    min_axis = min_axis - (max_axis - min_axis) / 10
    max_axis = (max_axis - min_axis) / 10 + max_axis
    xy_subplot.set_xlim(min_axis, max_axis)
    xy_subplot.set_ylim(min_axis, max_axis)
    xy_subplot.set_aspect('equal')
    
    # labels
    position_subplot.set_ylabel('x (m)')
    twin_position.set_ylabel('y (m)', rotation=270, labelpad=15)

    rotation_subplot.set_ylabel('yaw (deg)')

    command_subplot.set_xlabel('time (s)')
    command_subplot.set_ylabel('x (m/s)')
    twin_command.set_ylabel('yaw (rad/s)', rotation=270, labelpad=15)

    xy_subplot.set_xlabel('x (m)')
    xy_subplot.set_ylabel('y (m)')

    # legends
    position_subplot.legend(loc='upper left')
    twin_position.legend(loc='upper right')
    rotation_subplot.legend(loc='upper left')
    command_subplot.legend(loc='upper left')
    twin_command.legend(loc='upper right')
    xy_subplot.legend()

    # plot colorbar for xy subplot
    cbar = plt.colorbar(xy_subplot.collections[0], ax=xy_subplot)
    cbar.set_label('time (s)')
    cbar.ax.yaxis.label.set_color(Colors.light_grey)
    cbar.ax.tick_params(colors=Colors.light_grey)

    fig.set_tight_layout(True)
    plt.show()
    fig.savefig("telemetry.png", facecolor=fig.get_facecolor(), edgecolor='none')

if __name__ == "__main__":
    main()
