import optuna
from object_localization.global_registration import GlobalRegistration
import logging
import joblib


class GlobalRegistrationOptimizer():
    def __init__(self):
        self.study = optuna.create_study(direction='minimize')
        self._source_file = "../data/0_0_0_150.ply"
        self._target_files = [
            "0_0_0_150_2.ply",
            "0_0_0_250.ply",
            "0_m25_0_150.ply",
            "0_m25_0_250.ply",
            "0_m25_0_350.ply",
            "m50_25_0_150.ply",
        ]
        self.global_registration = GlobalRegistration(self._source_file, self._target_files[0])
        self.global_registration.set_log_level(logging.ERROR)



    def objective(self, trial: optuna.Trial):
        voxel_size = trial.suggest_float('voxel_size', 0.001, 0.01)
        normal_radius = trial.suggest_float('normal_radius', 0.0001, 0.01)
        search_radius = trial.suggest_float('search_radius', 0.001, 0.01)
        distance_threshold = trial.suggest_float('distance_threshold', 0.01, 1.0)
        max_nn_normal = trial.suggest_int('max_nn_normal', 1, 100)
        max_nn_fpfh = trial.suggest_int('max_nn_fpfh', 10, 500)

        self.global_registration.voxel_size = voxel_size
        self.global_registration.normal_radius = normal_radius
        self.global_registration.search_radius = search_radius
        self.global_registration.distance_threshold = distance_threshold
        self.global_registration.max_nn_normal = max_nn_normal
        self.global_registration.max_nn_fpfh = max_nn_fpfh

        objective = 0
        errors_rotation = []
        errors_translation = []
        times = []

        for target_file in self._target_files:
            target_file = "../data/" + target_file
            self.global_registration.target_file = target_file
            error_rotation, error_translation, time = self.global_registration.colored_registration()
            errors_rotation.append(error_rotation)
            errors_translation.append(error_translation)
            times.append(time)
        print(f"Average rotation error: {sum(errors_rotation)/len(errors_rotation)}")
        print(f"Average translation error: {sum(errors_translation)/len(errors_translation)}")
        print(f"Average time: {sum(times)/len(times)}")
        objective = 1 * sum(errors_rotation) + sum(errors_translation) + 0.0001 * sum(times)
        return objective/len(self._target_files)


def main():
    global_registration_optimizer = GlobalRegistrationOptimizer()
    global_registration_optimizer.study.optimize(global_registration_optimizer.objective, n_trials=50)
    # Store study results using joblib
    joblib.dump(global_registration_optimizer.study, "global_registration_study.pkl")


if __name__ == "__main__":
    main()
        

