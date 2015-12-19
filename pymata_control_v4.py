__author__ = 'mnapolitano'

from pymata_aio.constants import Constants
from pymata_aio.pymata3 import PyMata3
from yaml import load

testing = True


class Experiment:
    def __init__(self, expdict):
        self.init_dict = expdict
        self.experiment_values = {}
        self.sample_dict = {}
        self.board = None
        self.sample_list = []
        for key in expdict:
            print(key)
            if key == 'experiment_values':
                self.experiment_values = expdict[key]
            else:
                self.sample_dict[key] = expdict[key]

        try:
            self.com_port = self.experiment_values['boards']
            self.autodetect = False
        except KeyError:
            self.autodetect = True
        self.name = self.experiment_values['name']
        self.verbose = self.experiment_values['verbose']
        self.exptype = self.experiment_values['exptype']

        self.interactive_mode = self.experiment_values['interactive_mode']
        self.override_mix = self.experiment_values['override_mix']
        self.new_experiment = self.experiment_values['new_experiment']

        self.save_experiment = self.experiment_values['save_experiment']
        self.save_every = self.experiment_values['save_every']
        self.sample_save = self.experiment_values['sample_save']
        self.save_file = self.experiment_values['save_file']

        self.analog = self.experiment_values['analog']

        if not self.analog:
            self.i2c_addresses = self.experiment_values['i2c_addresses']

        self.plotly = self.experiment_values['plotly']
        if self.plotly:
            self.maxpoints = self.experiment_values['maxpoints']
            self.plotly_api_key = self.experiment_values['plotly_api_key']
            self.plotly_username = self.experiment_values['plotly_username']

        self.reads_normal = self.experiment_values['reads_normal']
        self.sleeptime_normal = self.experiment_values['sleeptime_normal']
        self.reads_diluting = self.experiment_values['reads_diluting']
        self.sleeptime_diluting = self.experiment_values['sleeptime_diluting']
        self.voltage_type = self.experiment_values['voltage_type']
        self.tolerance = self.experiment_values['tolerance']
        self.autovolt = self.experiment_values['autovolt']
        self.sampling_interval = self.experiment_values['sampling_interval']

        self.resolution = self.experiment_values['resolution']
        self.wiggle = self.experiment_values['wiggle']
        self.step_size = self.experiment_values['step_size']
        self.initial_adaptation_cycles = self.experiment_values['initial_adaptation_cycles']
        self.adaptation_cycles_to_step = self.experiment_values['adaptation_cycles_to_step']
        self.growth_rate_type = self.experiment_values['growth_rate_type']

        self.board_init()

        self.sample_init()

    def sample_init(self):
        for sample in self.sample_dict:
            self.sample_list.append(Sample(self, self.sample_dict[sample]))

    def board_init(self):
        if not testing:
            if not self.autodetect:
                self.board = PyMata3(com_port=self.com_port[0], arduino_wait=10)
            else:
                self.board = PyMata3(arduino_wait=10)


class Sample:
    def __init__(self, experiment_object, sample_dict):
        self.NEW_LOAD = 1
        self.settings = sample_dict
        self.experiment = experiment_object
        self.board = self.experiment.board
        self.name = self.settings['name']
        self.channel = self.settings['channel']
        self.analog_pin = self.settings['analog_pin']
        self.pump_rest_pin = self.settings['pump_restrictive_pin']
        self.pump_perm_pin = self.settings['pump_permissive_pin']
        self.pump_waste_pin = self.settings['pump_waste_pin']
        self.v_adj = self.settings['v_adj']
        self.growth_rate = self.settings['growth_rate']
        self.wt_growth_rate = self.growth_rate
        self.volt_on = self.settings['volt_on']
        self.volt_off = self.settings['volt_off']
        self.last_volt = 0
        self.growth_rate_history = []
        self.currmix = [1, 0]
        if self.experiment.autovolt:
            self.voltage_history = []
        if self.experiment.plotly:
            self.stream_id = self.settings['stream_id']

        self.pump_list = []
        self.pump_pins = [self.pump_rest_pin, self.pump_perm_pin, self.pump_waste_pin]

        self.pump_init()
        self.analog_init()

    def __getstate__(self):
        odict = self.__dict__.copy()
        del odict['experiment']
        return odict

    def __setstate__(self, state):
        print(state)
        self.__dict__ = state
        # self.sample_pump_init()
        # self.voltage_init()

    def pump_init(self):
        for pump in self.pump_pins:
            self.pump_list.append(Pump(pump, self))

    def analog_init(self):
        self.experiment.board.enable_analog_reporting(self.analog_pin)
        self.last_volt = self.voltage_average_n()
        print(self.last_volt)
        if not self.experiment.autovolt == 'TRUE':
            try:
                x = float(input("New Volt on? Current volt_on is: %f or zero to skip\n" % self.volt_on))
                if x == 0.0:
                    pass
                else:
                    self.volt_on = x
            except ValueError:
                print('input error, using current volt_on')

            try:
                y = float(input("New volt off? Current volt_off is: %f or zero to skip \n" % self.volt_off))
                if y == 0.0:
                    pass
                else:
                    self.volt_off = y
            except ValueError:
                print('input error, using current volt_off')
        else:
            print("Using autovolt, please start from very low dilution")

    def voltage_average_n(self):
        pass

    def voltage_average_d(self):
        pass

    def mix_override(self):
        self.growth_rate_history = []
        self.wt_growth_rate = 40000
        self.NEW_LOAD = 1
        try:
            y = float(input('New permissive fraction? (e.g. .80 for 80% permissive), -1 to use current'))
            if y == -1:
                pass
            else:
                self.currmix = [y, 1 - y]
        except ValueError:
            print("input error, using current mix")


class Pump:
    def __init__(self, pump_pin, sample_object):
        self.pump_pin = pump_pin
        self.sample = sample_object
        self.board = self.sample.experiment.board

        self.board.set_pin_mode(pump_pin, Constants.OUTPUT)
        self.pump_off()

    def pump_on(self):
        self.board.digital_write(self.pump_pin, 1)

    def pump_off(self):
        self.board.digital_write(self.pump_pin, 0)


class ExperimentBootstrap:
    def __init__(self):
        self.experiment_list = []
        with open('/Users/mnapolitano/Documents/Experiment1.yaml', 'r+') as yaml_file:
            loaded_yaml = load(yaml_file.read())
            for experiment in loaded_yaml:
                # print(experiment)
                experiment_dict = loaded_yaml[experiment]
                # print(experiment_dict)
                self.experiment_list.append(Experiment(experiment_dict))


test = ExperimentBootstrap()
