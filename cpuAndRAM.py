import psutil

def get_cpu_usage():
    # Get CPU usage percentage
    return psutil.cpu_percent(interval=1)  # Interval of 1 second

def get_ram_usage():
    # Get RAM usage information
    ram_info = psutil.virtual_memory()
    total_memory = ram_info.total
    used_memory = ram_info.used
    percent_used = ram_info.percent
    return total_memory, used_memory, percent_used

def get_cpu_temp():
    return psutil.sensors_temperatures()

if __name__ == "__main__":
    cpu_usage = get_cpu_usage()
    total_memory, used_memory, ram_usage_percent = get_ram_usage()
    cpu_temp = get_cpu_temp()
    #print(cpu_temp)
    # cpu_temp_core = cpu_temp['coretemp']
    # cpu_temp_core_overall = cpu_temp_core

    print(f"CPU Usage: {cpu_usage}%")
    print(f"Total RAM: {total_memory / (1024 ** 3):.2f} GB")
    print(f"Used RAM: {used_memory / (1024 ** 3):.2f} GB")
    print(f"RAM Usage: {ram_usage_percent}%")
    print(f"CPU Temperature: {cpu_temp['cpu_thermal'][0].current}C")

