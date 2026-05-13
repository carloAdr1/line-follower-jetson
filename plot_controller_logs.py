import csv
import sys
import os
import glob
import math
import matplotlib.pyplot as plt


def latest_log():
    files = glob.glob("logs/controller_log_*.csv")
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def moving_average(values, window=10):
    result = []
    for i in range(len(values)):
        start = max(0, i - window + 1)
        chunk = values[start:i + 1]
        result.append(sum(chunk) / len(chunk))
    return result


def moving_std(values, window=10):
    result = []
    for i in range(len(values)):
        start = max(0, i - window + 1)
        chunk = values[start:i + 1]
        mean = sum(chunk) / len(chunk)
        var = sum((x - mean) ** 2 for x in chunk) / len(chunk)
        result.append(math.sqrt(var))
    return result


def derivative(values, time):
    result = [0.0]
    for i in range(1, len(values)):
        dt = time[i] - time[i - 1]
        if dt <= 0:
            result.append(0.0)
        else:
            result.append((values[i] - values[i - 1]) / dt)
    return result


def stats(values):
    if not values:
        return {
            "mean": 0.0,
            "max": 0.0,
            "rmse": 0.0
        }

    mean = sum(values) / len(values)
    max_v = max(values)
    rmse = math.sqrt(sum(v ** 2 for v in values) / len(values))

    return {
        "mean": mean,
        "max": max_v,
        "rmse": rmse
    }


if len(sys.argv) >= 2:
    csv_path = sys.argv[1]
else:
    csv_path = latest_log()

if csv_path is None:
    print("No encontré logs. Primero corre controller_node.py y ciérralo con Ctrl+C.")
    sys.exit(1)

if not os.path.exists(csv_path):
    print(f"No existe el archivo: {csv_path}")
    sys.exit(1)

print(f"Usando log: {csv_path}")

data = {
    'time_s': [],
    'offset_px': [],
    'heading_deg': [],
    'control_estimated_v': [],
    'servo_level': [],
    'lane_error_abs_px': [],
    'heading_error_abs_deg': [],
    'stop_detected': [],
    'stop_active': [],
    'control_effort_abs': [],
    'servo_effort_abs': []
}

with open(csv_path, "r") as f:
    reader = csv.DictReader(f)

    for row in reader:
        for key in data:
            if key in row:
                data[key].append(float(row[key]))
            else:
                data[key].append(0.0)

out_dir = "plots"
os.makedirs(out_dir, exist_ok=True)

base = os.path.splitext(os.path.basename(csv_path))[0]
t = data['time_s']


def save_plot(y, title, ylabel, filename):
    plt.figure()
    plt.plot(t, y)
    plt.title(title)
    plt.xlabel("Tiempo [s]")
    plt.ylabel(ylabel)
    plt.grid(True)
    path = os.path.join(out_dir, filename)
    plt.savefig(path, dpi=200, bbox_inches="tight")
    plt.close()
    print(f"Guardado: {path}")


# Variables derivadas
offset_ma = moving_average(data['lane_error_abs_px'], window=10)
heading_ma = moving_average(data['heading_error_abs_deg'], window=10)

offset_std = moving_std(data['offset_px'], window=10)
heading_std = moving_std(data['heading_deg'], window=10)

offset_rate = derivative(data['offset_px'], t)
heading_rate = derivative(data['heading_deg'], t)

offset_rate_abs = [abs(v) for v in offset_rate]
heading_rate_abs = [abs(v) for v in heading_rate]

# 1. Error lateral
save_plot(
    data['offset_px'],
    "Offset vs Tiempo",
    "Offset [px]",
    f"{base}_01_offset_vs_tiempo.png"
)

# 2. Error angular
save_plot(
    data['heading_deg'],
    "Heading vs Tiempo",
    "Heading [deg]",
    f"{base}_02_heading_vs_tiempo.png"
)

# 3. Error absoluto lateral + promedio móvil
plt.figure()
plt.plot(t, data['lane_error_abs_px'], label="|offset|")
plt.plot(t, offset_ma, label="Promedio móvil |offset|")
plt.title("Evolución y convergencia del error lateral")
plt.xlabel("Tiempo [s]")
plt.ylabel("Error lateral [px]")
plt.grid(True)
plt.legend()
path = os.path.join(out_dir, f"{base}_03_convergencia_error_lateral.png")
plt.savefig(path, dpi=200, bbox_inches="tight")
plt.close()
print(f"Guardado: {path}")

# 4. Error angular absoluto + promedio móvil
plt.figure()
plt.plot(t, data['heading_error_abs_deg'], label="|heading|")
plt.plot(t, heading_ma, label="Promedio móvil |heading|")
plt.title("Evolución y convergencia del error angular")
plt.xlabel("Tiempo [s]")
plt.ylabel("Error angular [deg]")
plt.grid(True)
plt.legend()
path = os.path.join(out_dir, f"{base}_04_convergencia_error_angular.png")
plt.savefig(path, dpi=200, bbox_inches="tight")
plt.close()
print(f"Guardado: {path}")

# 5. Control estimado
save_plot(
    data['control_estimated_v'],
    "Control estimado continuo",
    "v = offset + heading*K",
    f"{base}_05_control_estimado.png"
)

# 6. Servo discreto
save_plot(
    data['servo_level'],
    "Comando discreto del servo",
    "Servo level [-2, -1, 0, 1, 2]",
    f"{base}_06_servo_level.png"
)

# 7. Antes/después del servo
plt.figure()
plt.plot(t, data['control_estimated_v'], label="Antes: control continuo estimado")
plt.plot(t, data['servo_level'], label="Después: comando discreto al servo")
plt.title("Comparación antes/después del servo")
plt.xlabel("Tiempo [s]")
plt.ylabel("Valor")
plt.grid(True)
plt.legend()
path = os.path.join(out_dir, f"{base}_07_antes_despues_servo.png")
plt.savefig(path, dpi=200, bbox_inches="tight")
plt.close()
print(f"Guardado: {path}")

# 8. Esfuerzo de control
plt.figure()
plt.plot(t, data['control_effort_abs'], label="|control_v|")
plt.plot(t, data['servo_effort_abs'], label="|servo_level|")
plt.title("Esfuerzo de control")
plt.xlabel("Tiempo [s]")
plt.ylabel("Magnitud")
plt.grid(True)
plt.legend()
path = os.path.join(out_dir, f"{base}_08_esfuerzo_control.png")
plt.savefig(path, dpi=200, bbox_inches="tight")
plt.close()
print(f"Guardado: {path}")

# 9. Estabilidad lateral
save_plot(
    offset_std,
    "Estabilidad lateral: desviación móvil del offset",
    "Std móvil offset [px]",
    f"{base}_09_estabilidad_offset.png"
)

# 10. Estabilidad angular
save_plot(
    heading_std,
    "Estabilidad angular: desviación móvil del heading",
    "Std móvil heading [deg]",
    f"{base}_10_estabilidad_heading.png"
)

# 11. Sensibilidad a perturbaciones laterales
save_plot(
    offset_rate_abs,
    "Sensibilidad a perturbaciones: cambio del offset",
    "|d(offset)/dt| [px/s]",
    f"{base}_11_perturbaciones_offset.png"
)

# 12. Sensibilidad a perturbaciones angulares
save_plot(
    heading_rate_abs,
    "Sensibilidad a perturbaciones: cambio del heading",
    "|d(heading)/dt| [deg/s]",
    f"{base}_12_perturbaciones_heading.png"
)

# 13. STOP detectado
save_plot(
    data['stop_detected'],
    "Detección de STOP",
    "STOP detectado [0/1]",
    f"{base}_13_stop_detectado.png"
)

# 14. STOP activo
save_plot(
    data['stop_active'],
    "Alto activo por STOP",
    "STOP activo [0/1]",
    f"{base}_14_stop_activo.png"
)

# 15. Offset + STOP activo
plt.figure()
plt.plot(t, data['offset_px'], label="Offset [px]")
plt.plot(t, data['stop_active'], label="STOP activo")
plt.title("Comportamiento de línea durante evento STOP")
plt.xlabel("Tiempo [s]")
plt.ylabel("Valor")
plt.grid(True)
plt.legend()
path = os.path.join(out_dir, f"{base}_15_offset_y_stop.png")
plt.savefig(path, dpi=200, bbox_inches="tight")
plt.close()
print(f"Guardado: {path}")

# 16. Resumen estadístico en TXT
lane_stats = stats(data['lane_error_abs_px'])
heading_stats = stats(data['heading_error_abs_deg'])
control_stats = stats(data['control_effort_abs'])
servo_stats = stats(data['servo_effort_abs'])
offset_rate_stats = stats(offset_rate_abs)
heading_rate_stats = stats(heading_rate_abs)

stop_events = sum(
    1 for i in range(1, len(data['stop_detected']))
    if data['stop_detected'][i - 1] == 0 and data['stop_detected'][i] == 1
)

stop_active_time = 0.0
for i in range(1, len(t)):
    if data['stop_active'][i] > 0.5:
        stop_active_time += t[i] - t[i - 1]

summary_path = os.path.join(out_dir, f"{base}_00_resumen_metricas.txt")

with open(summary_path, "w") as f:
    f.write("Resumen cuantitativo del controlador visual\n")
    f.write("==========================================\n\n")

    f.write(f"Archivo analizado: {csv_path}\n")
    f.write(f"Duración total: {t[-1]:.2f} s\n")
    f.write(f"Muestras: {len(t)}\n\n")

    f.write("1. Error lateral de línea\n")
    f.write(f"MAE |offset|: {lane_stats['mean']:.3f} px\n")
    f.write(f"RMSE offset: {lane_stats['rmse']:.3f} px\n")
    f.write(f"Máximo |offset|: {lane_stats['max']:.3f} px\n\n")

    f.write("2. Error angular\n")
    f.write(f"MAE |heading|: {heading_stats['mean']:.3f} deg\n")
    f.write(f"RMSE heading: {heading_stats['rmse']:.3f} deg\n")
    f.write(f"Máximo |heading|: {heading_stats['max']:.3f} deg\n\n")

    f.write("3. Esfuerzo de control\n")
    f.write(f"Promedio |control_v|: {control_stats['mean']:.3f}\n")
    f.write(f"Máximo |control_v|: {control_stats['max']:.3f}\n")
    f.write(f"Promedio |servo_level|: {servo_stats['mean']:.3f}\n")
    f.write(f"Máximo |servo_level|: {servo_stats['max']:.3f}\n\n")

    f.write("4. Sensibilidad a perturbaciones\n")
    f.write(f"Promedio |d(offset)/dt|: {offset_rate_stats['mean']:.3f} px/s\n")
    f.write(f"Máximo |d(offset)/dt|: {offset_rate_stats['max']:.3f} px/s\n")
    f.write(f"Promedio |d(heading)/dt|: {heading_rate_stats['mean']:.3f} deg/s\n")
    f.write(f"Máximo |d(heading)/dt|: {heading_rate_stats['max']:.3f} deg/s\n\n")

    f.write("5. STOP\n")
    f.write(f"Eventos STOP detectados: {stop_events}\n")
    f.write(f"Tiempo total en alto: {stop_active_time:.3f} s\n\n")

    f.write("Interpretación sugerida\n")
    f.write("-----------------------\n")
    f.write(
        "El offset y el heading representan los errores visuales principales. "
        "La disminución o acotamiento de sus magnitudes indica comportamiento de convergencia. "
        "El control_v representa la acción continua estimada antes de discretizarse en comandos de servo. "
        "El servo_level representa la acción real enviada al sistema de dirección. "
        "Las desviaciones móviles permiten discutir estabilidad, mientras que las derivadas absolutas "
        "permiten analizar sensibilidad ante perturbaciones visuales o cambios bruscos en la pista. "
        "Las señales stop_detected y stop_active muestran la integración del señalamiento STOP dentro "
        "del lazo de control visual.\n"
    )

print(f"Guardado: {summary_path}")
print("Listo. Gráficas y resumen guardados en plots/")
