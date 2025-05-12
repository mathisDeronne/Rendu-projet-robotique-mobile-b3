import tkinter as tk
import subprocess
import sys
import os

# === Définition des modes ===
MODES_SIM2 = ["direct", "inverse", "triangle", "circle", "segment"]
MODES_HEXA = ["marche", "tourne_droite", "tourne_gauche", "danse_stickbug", "target position", "Standing still animation"]
MODES_REAL = ["squat", "av_ar", "dance"]

CONSTANTS_PATH = "constants.py"
real_process = None  # Pour suivre le processus du mode réel

# === Fonctions générales ===
def set_robot_type(new_type):
    with open(CONSTANTS_PATH, "r", encoding="utf-8") as f:
        lines = f.readlines()

    updated = False
    with open(CONSTANTS_PATH, "w", encoding="utf-8") as f:
        for line in lines:
            if line.strip().startswith("ROBOT_TYPE ="):
                current_type = line.strip().split("=")[1].strip()
                if current_type != new_type:
                    f.write(f"ROBOT_TYPE = {new_type}\n")
                    updated = True
                else:
                    f.write(line)
            else:
                f.write(line)

    if updated:
        print(f"[INFO] ROBOT_TYPE updated to {new_type}")
    else:
        print(f"[INFO] ROBOT_TYPE already set to {new_type}")

def clear_mode_buttons():
    for widget in mode_frame.winfo_children():
        widget.destroy()

# === Fonctions spécifiques ===
def show_sim2_modes():
    set_robot_type("ARM_SIMULATION")
    clear_mode_buttons()

    label = tk.Label(mode_frame, text="Modes Bras Articulé", font=("Helvetica", 13), bg="#f0f0f0")
    label.pack(pady=10)

    for mode in MODES_SIM2:
        btn = tk.Button(mode_frame, text=mode, width=25, bg="#2196F3", fg="white",
                        command=lambda m=mode: subprocess.Popen([sys.executable, "sim2.py", "--mode", m]))
        btn.pack(pady=5)

def show_hexa_modes():
    set_robot_type("PHANTOMX_SIMULATION")
    clear_mode_buttons()

    label = tk.Label(mode_frame, text="Modes Hexapode", font=("Helvetica", 13), bg="#f0f0f0")
    label.pack(pady=10)

    for mode in MODES_HEXA:
        btn = tk.Button(mode_frame, text=mode, width=25, bg="#4CAF50", fg="white",
                        command=lambda m=mode: subprocess.Popen([sys.executable, "main_hexa.py", "--mode", m]))
        btn.pack(pady=5)

def show_real_modes():
    set_robot_type("PHANTOMX_REAL")
    clear_mode_buttons()

    label = tk.Label(mode_frame, text="Modes Réels", font=("Helvetica", 13), bg="#f0f0f0")
    label.pack(pady=10)

    for mode in MODES_REAL:
        btn = tk.Button(mode_frame, text=mode, width=25, bg="#FF5722", fg="white",
                        command=lambda m=mode: launch_real_mode(m))
        btn.pack(pady=5)

    stop_btn = tk.Button(mode_frame, text="⛔ Arrêter le mode actuel", width=25, bg="red", fg="white",
                         command=stop_real_mode)
    stop_btn.pack(pady=20)

def launch_real_mode(mode):
    global real_process

    if real_process is not None and real_process.poll() is None:
        real_process.terminate()
        real_process.wait()

    real_process = subprocess.Popen([sys.executable, "main_reel.py", mode])
    print(f"[INFO] Lancement du mode réel : {mode}")

def stop_real_mode():
    global real_process

    if real_process is not None and real_process.poll() is None:
        real_process.terminate()
        real_process.wait()
        real_process = None
        print("[INFO] Programme réel arrêté.")

# === Interface ===
root = tk.Tk()
root.title("Interface de Simulation")
root.geometry("600x400")
root.configure(bg="#f0f0f0")

main_frame = tk.Frame(root, bg="#f0f0f0")
main_frame.pack(fill="both", expand=True, padx=20, pady=20)

robot_frame = tk.Frame(main_frame, bg="#f0f0f0")
robot_frame.pack(side="left", fill="y", padx=(0, 20))

label = tk.Label(robot_frame, text="Choisissez un robot :", font=("Helvetica", 14), bg="#f0f0f0")
label.pack(pady=10)

btn_sim2 = tk.Button(robot_frame, text="Bras Articulé", width=20, bg="#2196F3", fg="white", command=show_sim2_modes)
btn_sim2.pack(pady=10)

btn_hexa = tk.Button(robot_frame, text="Hexapode", width=20, bg="#4CAF50", fg="white", command=show_hexa_modes)
btn_hexa.pack(pady=10)

btn_real = tk.Button(robot_frame, text="Mode Réel", width=20, bg="#FF5722", fg="white", command=show_real_modes)
btn_real.pack(pady=10)

mode_frame = tk.Frame(main_frame, bg="#f0f0f0")
mode_frame.pack(side="left", fill="both", expand=True)

root.mainloop()
