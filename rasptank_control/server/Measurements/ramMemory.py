#!/usr/bin/python3
# File name   : Measurements/ramMemory.py
# Description : Read ram use
# Author      : Milosz Plutowski
# Version     : v1.0
# Date        : 2021/08/21

import psutil
import json

# data dictionary
dict_data = {}

'''
    @brief Reading CPU temperature
    @return - using ram
'''
def get_ram_info():
    """ Return RAM usage using psutil """
    ram_cent = psutil.virtual_memory()[2]
    return str(ram_cent)

if __name__ == '__main__':
    useRAM = get_ram_info()
    useRAM = round(float(useRAM)*18.5,2)
    dict_data['name'] = 'memory'
    dict_data['value'] = useRAM
    dict_data['unit'] = 'MB'
    temp = dict_data
    temp_json = json.dumps(temp)
    print(temp_json)