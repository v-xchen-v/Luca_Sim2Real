def setup_scene():
    pass


def main():
    setup_scene()
    for i in range(10):
        grasp_and_place = GraspAndPlace('trace.npy')
        grasp_and_place.run()