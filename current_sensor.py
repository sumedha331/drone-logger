import serial
import openpyxl
import matplotlib.pyplot as plt
import datetime
import os

SERIAL_PORT = 'COM7'
BAUD_RATE = 9600
MAX_ROWS = 10000
EXCEL_DIR = 'logs'

# Create logs directory
if not os.path.exists(EXCEL_DIR):
    os.makedirs(EXCEL_DIR)

timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
file_path = os.path.join(EXCEL_DIR, f"DroneData_{timestamp}.xlsx")
wb = openpyxl.Workbook()
ws = wb.active
ws.title = "Data"
ws.append(["Thrust (g)", "Current (A)", "PWM (us)"])

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

plt.ion()
fig, ax = plt.subplots()
thrust_data = []
current_data = []

print("\n[INFO] Logging started. Press Ctrl+C to stop.\n")

try:
    while len(thrust_data) < MAX_ROWS:
        line = ser.readline().decode(errors="ignore").strip()
        if line.count(',') == 2:
            try:
                thrust, current, pwm = map(float, line.split(','))
                thrust_data.append(thrust)
                current_data.append(current)
                ws.append([thrust, current, pwm])

                ax.clear()
                colors = ['r' if thrust_data[i] < thrust_data[i-1] else 'b' for i in range(1, len(thrust_data))]
                colors.insert(0, 'b')

                for i in range(1, len(thrust_data)):
                    ax.plot(current_data[i-1:i+1], thrust_data[i-1:i+1], f'{colors[i]}.-')

                ax.set_xlabel("Current (A)")
                ax.set_ylabel("Thrust (g)")
                ax.set_title("Thrust vs Current")
                ax.grid(True)
                plt.pause(0.01)

            except ValueError:
                continue

except KeyboardInterrupt:
    print("\n[INFO] Logging stopped by user.")

finally:
    print("[INFO] Saving Excel and cleaning up...")
    wb.save(file_path)
    ser.close()
    plt.ioff()

    plot_path = os.path.join(EXCEL_DIR, f"Plot_{timestamp}.png")
    fig.savefig(plot_path)
    print(f"[INFO] Plot saved to: {plot_path}")

    plt.show()
    print(f"\n[INFO] Data saved to: {file_path}")