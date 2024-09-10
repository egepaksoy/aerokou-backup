import serial
import time

# Open serial connection to the modem (adjust the port and baudrate)
ser = serial.Serial('com7', 57600, timeout=1)

# Enter command mode
ser.write(b'+++')
time.sleep(1)  # Ensure there's a pause before/after the command

# Reset to factory defaults
ser.write(b'AT&F\r\n')

# Save the settings
ser.write(b'AT&W\r\n')

# Reboot the modem
ser.write(b'ATZ\r\n')

# Close the serial connection
ser.close()
