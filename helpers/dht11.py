import os

# Path to the kernel IIO (Industrial I/O) device files
DHT11_DIR = "/sys/bus/iio/devices/iio:device0"
TEMP_FILE = os.path.join(DHT11_DIR, "in_temp_input")
HUMID_FILE = os.path.join(DHT11_DIR, "in_humidityrelative_input")

# the dht11 pin si PIN40

def read_kernel_sensor(file_path):
  try:
    if os.path.exists(file_path):
      with open(file_path, "r") as f:
        raw_val = f.read().strip()
        # The kernel returns values multiplied by 1000 (e.g., 25000 for 25.0°C)
        return float(raw_val) / 1000.0
  except IOError:
      # This catches moments when the kernel driver is mid-refresh or misses a pulse
      return None
  return None
  
def readHumidityAndTemp():
  temperature = read_kernel_sensor(TEMP_FILE)
  humidity = read_kernel_sensor(HUMID_FILE)
  return temperature, humidity
  

__all__ = ["readHumidityAndTemp"]
  