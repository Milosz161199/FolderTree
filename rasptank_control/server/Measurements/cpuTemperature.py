#!/usr/bin/python3
# File name   : Measurements/cpuTemperature.py
# Description : Read cpu temperature
# Author      : Milosz Plutowski
# Version     : v1.0
# Date        : 2021/08/21

import json

# data dictionary
dict_data = {}

'''
    @brief Reading CPU temperature
    @return - cpu temperature
'''
def get_cpu_tempfunc():
    result = 0
    mypath = "/sys/class/thermal/thermal_zone0/temp"
    with open(mypath, 'r') as mytmpfile:
        for line in mytmpfile:
            result = line

    result = float(result)/1000
    result = round(result, 1)
    return str(result)
    
if __name__ == '__main__':
    cpuTemp = get_cpu_tempfunc()
    dict_data['name'] = 'temperature'
    dict_data['value'] = round(float(cpuTemp),2)
    dict_data['unit'] = 'C'
    temp = dict_data
    temp_json = json.dumps(temp)
    print(temp_json)