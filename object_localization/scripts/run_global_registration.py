import sys
from object_localization.global_registration import GlobalRegistration
import joblib
import logging

if __name__ == "__main__":

# Load the study from the file
    study = joblib.load("global_registration_study.pkl")
    best_params = study.best_params
    print(best_params)


    global_registration = GlobalRegistration(sys.argv[1], sys.argv[2])

    global_registration.set_log_level(logging.DEBUG)
    global_registration.voxel_size = best_params['voxel_size']
    global_registration.normal_radius = best_params['normal_radius']
    global_registration.search_radius = best_params['search_radius']
    global_registration.distance_threshold = best_params['distance_threshold']
    global_registration.max_nn_normal = best_params['max_nn_normal']
    global_registration.max_nn_fpfh = best_params['max_nn_fpfh']

    global_registration.colored_registration()
