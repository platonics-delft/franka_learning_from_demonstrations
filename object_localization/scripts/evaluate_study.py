import optuna
from optuna.visualization import plot_optimization_history, plot_param_importances
import joblib

# Load the study from the file
study = joblib.load("global_registration_study.pkl")
fig1 = plot_optimization_history(study)
fig2 = plot_param_importances(study)
fig1.show()
fig2.show()

print(study.best_params)


