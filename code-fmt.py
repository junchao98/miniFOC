#!/bin/python3

import os
import sys

# 自动去除代码中无用的空格和tab
file_name = sys.argv[1];

file = open(file_name, mode='r');
lines = [];
count = 0;

while True:
    
    line = file.readline();
    count += 1;
    if not line:
        print("end line: {}".format(count));
        break;
    
    line = line.rstrip('\n')

    if line == '\t':
        print("check tab in {}".format(count));
        continue;

    s_cnt = line.count(' ')
    if (s_cnt == len(line)) and (len(line) != 0):
        print("check space in {}".format(count));
        continue;

    line = line.rstrip(' ');
    line = line.rstrip('\t');

    line += '\n'
    lines.append(line);

file.close()

if (len(sys.argv) >2) and (sys.argv[2] == "keep"):
    file = open(file_name+".out", mode='w');
else:
    file = open(file_name, mode='w');

for line in lines:
    file.write(line);

file.close()

