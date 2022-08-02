import sys
sys.path.insert(1, '..')
from Thesis.Experiment.analysis import Analysis


# Plot stuff
analysis = Analysis()
analysis.initialize()
analysis.analyse()
print("Finished!")