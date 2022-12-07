import base64

import csb_sim
import imageio
import numpy as np
from matplotlib import pyplot as plt

from new.helper import plot_current_frame, convert_to_gif


def main():
    simulation = csb_sim.sim()

    loop = True
    plot = True
    images = []

    while loop:
        for i, pod in enumerate(simulation.pods):
            cp = simulation.checkpoints[pod.ncpid]
            raw_angle_next_cp = pod.diff_angle(cp)
            angle = max(-18, min(18, raw_angle_next_cp))

            pod.apply(100, angle)

        simulation.on_round()

        if plot:
            images.append(plot_current_frame(simulation.checkpoints, simulation.pods, components))

        for pod in simulation.pods:
            if pod.checked == int(len(simulation.checkpoints) * simulation.laps) or pod.timeout < 0:
                loop = False
                break

        # break
    if plot:
        convert_to_gif(f"test", images)


def plot_current_frame(checkpoints, pods, components):
    fig, ax = plt.subplots(figsize=(4, 3))
    ax.set_xlim((0, 16000))
    ax.set_ylim((0, 9000))
    ax.axis('off')

    rgb_map = {
        0: "green",
        1: "orange"
    }
    for j, checkpoint in enumerate(checkpoints):
        ax.add_patch(plt.Circle((checkpoint.x, checkpoint.y), 400, color="b", alpha=0.8))

    _tmp = {}
    for i, pod in enumerate(pods):
        for j, checkpoint in enumerate(checkpoints):
            alpha = 1
            if checkpoint.id == pod.ncpid:
                if checkpoint.id not in _tmp:
                    _tmp[checkpoint.id] = 0
                _tmp[checkpoint.id] += 100
                color = rgb_map[i]
                ax.add_patch(
                    plt.Circle((checkpoint.x, checkpoint.y), 500 - _tmp[checkpoint.id], color=color, alpha=alpha))
        ax.add_patch(plt.Circle((pod.x, pod.y), 300, color=rgb_map[i], label=components[i].name))

    ax.legend()
    fig.canvas.draw()
    image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
    image = image.reshape(fig.canvas.get_width_height()[::-1] + (3,))

    plt.close()
    return image


def convert_to_gif(name: str, frames: list):
    file_name = f'./{name}.gif'
    imageio.mimsave(file_name, frames, fps=7)
    b64 = convert_to_base64(file_name)
    return b64


def convert_to_base64(file_name):
    with open(file_name, "rb") as file:
        return base64.b64encode(file.read()).decode("utf-8")


if __name__ == '__main__':
    main()
