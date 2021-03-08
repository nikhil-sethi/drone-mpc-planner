#!/usr/bin/env python2

import pykst as kst

file = "/home/ludwig/pats/base/build-vscode/logging/log.csv"

with open(file, 'r') as fh:
    field_names = fh.readline()

field_names = field_names.replace(" ", "").split(';')[:-1]

client = kst.Client(file)
client.clear()
client.hide_window()
client.set_datasource_option("Column Delimiter", ";", 0)
client.set_datasource_option("Column Type", 2, 0)
client.set_datasource_option("Data Start", 2-1, 0)
client.set_datasource_option("Fields Line", 1-1, 0)
client.set_datasource_option("Read Fields", True, 0)
client.set_datasource_option("Units Line", 2-1, 0)
client.set_datasource_option("Read Units", False, 0)

data_vectors = []
for i in range(len(field_names)):
    dv = client.new_data_vector(file, field_names[i])
    data_vectors.append(dv)

# Because the kst2 API is super crappy!
def data_vector(name):   
    for i in range(len(data_vectors)):
        if(data_vectors[i].field() == name):
            return data_vectors[i]

def vector_index(name):
    for i in range(len(data_vectors)):
        if(data_vectors[i].field() == name):
            return data_vectors[i]

color_bg = '#eeeeec'
color_bg = '#a0a0a4'
color_bg = '#babdb6'
color_bg = '#d3d7cf'
color_bg = '#2e3436'

color_x = '#3465ae'
color_y = '#cc0000'
color_z = '#4e9a06'
color_dim4 = '#5c3566'

elapsed = data_vector("elapsed")
ctrl_target_x = data_vector("ctrl_target_x")
ctrl_target_y = data_vector("ctrl_target_y")
ctrl_target_z = data_vector("ctrl_target_z")
target_pos_x = data_vector("target_pos_x")
target_pos_y = data_vector("target_pos_y")
target_pos_z = data_vector("target_pos_z")
batt_cell_v = data_vector("batt_cell_v")
pos_x = data_vector("posX_drone")
pos_y = data_vector("posY_drone")
pos_z = data_vector("posZ_drone")

c11 = client.new_curve(elapsed, ctrl_target_x)
c12 = client.new_curve(elapsed, ctrl_target_y)
c13 = client.new_curve(elapsed, ctrl_target_z)
c14 = client.new_curve(elapsed, target_pos_x)
c15 = client.new_curve(elapsed, target_pos_y)
c16 = client.new_curve(elapsed, target_pos_z)
c17 = client.new_curve(elapsed, pos_x)
c18 = client.new_curve(elapsed, pos_y)
c19 = client.new_curve(elapsed, pos_z)

c11.set_color(color_x)
c12.set_color(color_y)
c13.set_color(color_z)
c14.set_color(color_x)
c15.set_color(color_y)
c16.set_color(color_z)
c17.set_color(color_x)
c18.set_color(color_y)
c19.set_color(color_z)


c14.set_line_style(3)
c15.set_line_style(3)
c16.set_line_style(3)

#b1 = client.new_box()
p1 = client.new_plot(fill_color=color_bg)

p1.add(c11)
p1.add(c12)
p1.add(c13)
p1.add(c14)
p1.add(c15)
p1.add(c16)
p1.add(c17)
p1.add(c18)
p1.add(c19)
client.new_legend(p1)

c7 = client.new_curve(elapsed, batt_cell_v)
c7.set_color('#eeeeec')
p2 = client.new_plot(fill_color=color_bg)
p2.add(c7)


auto_throttle = data_vector("autoThrottle")
auto_roll = data_vector("autoRoll")
auto_pitch = data_vector("autoPitch")
auto_yaw = data_vector("autoYaw")
c31 = client.new_curve(elapsed, auto_throttle)
c32 = client.new_curve(elapsed, auto_roll)
c33 = client.new_curve(elapsed, auto_pitch)
c34 = client.new_curve(elapsed, auto_yaw)
c31.set_color(color_y)
c32.set_color(color_x)
c33.set_color(color_z)
c34.set_color(color_dim4)

p3 = client.new_plot(fill_color=color_bg)
p3.add(c31)
p3.add(c32)
p3.add(c33)
p3.add(c34)

client.new_legend(p3)

client.show_window()
