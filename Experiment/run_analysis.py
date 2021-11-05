import sys
sys.path.insert(1, '..')
from Experiment.analysis import Analysis


# Plot stuff
analysis = Analysis()
analysis.initialize()
analysis.analyse()
print("Finished!")