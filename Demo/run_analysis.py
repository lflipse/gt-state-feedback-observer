import sys
sys.path.insert(1, '..')
from Demo.analysis import Analysis


# Plot stuff
analysis = Analysis()
analysis.initialize()
analysis.analyse()