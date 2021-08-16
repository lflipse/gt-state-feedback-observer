from Experiment.plots import PlotStuff

class Analysis():
    def __init__(self):
        # Unpack data
        self.data = {}
        self.unpack_data()
        self.plot_stuff = PlotStuff()


    def unpack_data(self):
        self.data = {}

    def analyse(self):
        self.data = {}

    def show(self, participant):
        self.data[participant] = {}
