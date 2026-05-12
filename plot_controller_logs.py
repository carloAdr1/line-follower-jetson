import csv
import sys
import os
import matplotlib.pyplot as plt

if len(sys.argv) < 2:
    print("Uso:")
    print("python3 plot_controller_logs.py logs/controller_log_XXXX.csv")
    sys.exit(1)

csv_path = sys.argv[1]

data = {
    'time_s': [],
    'offset_px': [],
    'heading_deg': [],
    'control_estimated_v': [],
    'lane_error_abs_px': [],
    'heading_error_abs_deg': [],
    'stop_detected': [],
    'stop_active': []
}

with open(csv_path, "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        for key in data:
            data[key].append(float(row[key]))

out_dir = "plots"
os.makedirs(out_dir, exist_ok=True)

base = os.path.splitext(os.path.basename(csv_path))[0]

def save_plot(x, y, title, xlabel, ylabel, filename):
    plt.figure()
    plt.plot(x, y)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid(True)
    path = os.path.join(out_dir, filename)
    plt.savefig(path, dpi=200, bbox_inches="tight")
    print(f"Guardado: {path}")

t = data['time_s']

save_plot(
    t,
    data['offset_px'],
    "Error lateral de línea",
    "Tiempo [s]",
    "Offset [px]",
    f"{base}_lane_offset.png"
)

save_plot(
    t,
    data['heading_deg'],
    "Error angular de línea",
    "Tiempo [s]",
    "Heading [deg]",
    f"{base}_heading_error.png"
)

save_plot(
    t,
    data['control_estimated_v'],
    "Control estimado continuo",
    "Tiempo [s]",
    "v = offset + heading*K",
    f"{base}_control_estimated.png"
)

save_plot(
    t,
    data['lane_error_abs_px'],
    "Magnitud del error lateral",
    "Tiempo [s]",
    "|offset| [px]",
    f"{base}_lane_abs_error.png"
)

save_plot(
    t,
    data['heading_error_abs_deg'],
    "Magnitud del error angular",
    "Tiempo [s]",
    "|heading| [deg]",
    f"{base}_heading_abs_error.png"
)

save_plot(
    t,
    data['stop_detected'],
    "Detección de STOP",
    "Tiempo [s]",
    "STOP detectado",
    f"{base}_stop_detected.png"
)

save_plot(
    t,
    data['stop_active'],
    "Activación de alto por STOP",
    "Tiempo [s]",
    "STOP activo",
    f"{base}_stop_active.png"
)

print("Listo.")