import pandas as pd
import numpy as np
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

def __draw_enum_timeline(ax, t, values, labels, colors=None, title="", ylabel="", alpha=0.2):
    import matplotlib.patches as mpatches

    values = values.reset_index(drop=True)  # Por si vienen con índices raros
    t = t.reset_index(drop=True)

    if colors:
        prev_val = int(values.iloc[0])
        start_time = t.iloc[0]

        for i in range(1, len(values)):
            current_val = int(values.iloc[i])
            if current_val != prev_val:
                if prev_val < len(colors):
                    ax.axvspan(start_time, t.iloc[i], color=colors[prev_val], alpha=alpha)
                start_time = t.iloc[i]
                prev_val = current_val

        if prev_val < len(colors):
            ax.axvspan(start_time, t.iloc[-1], color=colors[prev_val], alpha=alpha)

    # Step plot
    ax.step(t, values, where="post", color="black", linewidth=0.8)
    ax.set_yticks(range(len(labels)))
    ax.set_yticklabels(labels)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True)

    if colors:
        patches = [
            mpatches.Patch(color=colors[i], label=labels[i])
            for i in range(len(labels))
        ]
        ax.legend(handles=patches, loc="upper right", fontsize="small")



def plot_motor_log(df,mode="manual"):    
    wrapped_deg = df["pos_deg"].values
    unwrapped_rad = np.unwrap(np.deg2rad(wrapped_deg))
    unwrapped_deg = np.rad2deg(unwrapped_rad)

    df["position_unwrapped_deg"] = unwrapped_deg

    # Crear figura con 4 subplots verticales
    fig, axes = plt.subplots(6, 1, figsize=(18, 16), sharex=True)

    # Posición envuelta vs unwrapped
    axes[0].plot(df["t"], df["pos_deg"], label="Position")
    #axes[0].plot(df["t"], df["position_unwrapped_deg"], label="Unwrapped")

    if mode == "manual":
        axes[0].plot(df["t"], df["manual_target"],color='red',label="Target")
    elif mode == "scan":
        axes[0].plot(df["t"], df["scan_initial"],label="Scan initial angle")
        axes[0].plot(df["t"], df["scan_end"],label="Scan end angle")

    axes[0].set_ylabel("Position (deg)")
    axes[0].set_ylim(0,360.0)
    axes[0].set_xlabel("Time [s]")
    axes[0].set_title("Position Tracking")
    axes[0].legend()
    axes[0].grid(True)

    # Velocidad
    axes[1].plot(df["t"], df["vel_deg_s"])
    axes[1].set_ylabel("Velocity (deg/s)")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_title("Velocity")
    axes[1].grid(True)

    # Throttle
    axes[2].plot(df["t"], df["throttle"])
    axes[2].set_ylabel("Throttle")
    axes[2].set_xlabel("Time [s]")
    axes[2].set_title("Throttle")
    axes[2].grid(True)

    # CMD
    motor_cmd_labels = [
    "STOP", "ALTA_CW", "MEDIA_CW", "BAJA_CW",
    "BAJA_CCW", "MEDIA_CCW", "ALTA_CCW"
    ]
    motor_cmd_colors = [
        "#cccccc", "#ff0000", "#ff6666", "#ffaaaa",
        "#aaaaff", "#6666ff", "#0000ff"
    ]
    __draw_enum_timeline(
        ax=axes[3],
        t=df["t"],
        values=df["cmd"],
        labels=motor_cmd_labels,
        colors=motor_cmd_colors,
        title="Command",
        ylabel="Command"
    )

    # Mode
    mode_labels = ["STOPPED", "MANUAL", "SCANNING", "ROTATING"]
    __draw_enum_timeline(
        ax=axes[4],
        t=df["t"],
        values=df["mode"],
        labels=mode_labels,
        title="Mode",
        ylabel="Mode"
    )

    # Scan state
    scan_state_labels = ["GOTO_START", "SCANNING_FORWARD", "SCANNING_BACKWARD"]
    __draw_enum_timeline(
        ax=axes[5],
        t=df["t"],
        values=df["scan_state"],
        labels=scan_state_labels,
        title="Scan state",
        ylabel="Scan state"
    )

    plt.tight_layout()
    plt.show()