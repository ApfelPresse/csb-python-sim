# from helper import plot_current_frame, convert_to_gif

from new.helper import plot_current_frame, convert_to_gif
# from new.solutions import kev
# import csb_sim as csb
#from csb_sim import sim
import csb_sim

# from solutions import kev, jonny


def main():

    simulation = csb_sim.sim()

    loop = True
    plot = True
    images = []

    components = {
        0: kev,
        1: kev
    }

    while loop:
        for i, pod in enumerate(simulation.pods):
            angle, thrust = components[i].handle(pod, simulation.pods, simulation.checkpoints)
            pod.apply(thrust, angle)

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



if __name__ == '__main__':
    main()
