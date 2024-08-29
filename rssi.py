import subprocess

def get_rssi(interface):
    try:
        # Run the iwconfig command
        result = subprocess.run(['iwconfig', interface], capture_output=True, text=True)
        output = result.stdout

        # Find the line that contains 'Signal level'
        for line in output.split('\n'):
            if 'Signal level' in line:
                # Extract the signal level
                parts = line.split('Signal level=')
                if len(parts) > 1:
                    signal_level = parts[1].split()[0]
                    return signal_level
        return None
    except Exception as e:
        print(f"An error occurred: {e}")
        return None

# Replace 'wlan0' with your actual wireless interface name
interface = 'wlo1'
rssi = get_rssi(interface)
if rssi:
    print(f"RSSI Value: {rssi}")
else:
    print("Could not retrieve RSSI value.")