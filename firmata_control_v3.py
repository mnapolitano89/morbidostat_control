__author__ = 'Michael'

import csv
import datetime
import pickle
import time

import plotly.plotly as py
import plotly.tools as tls
from PyMata.pymata import PyMata
# import pandas as pd
import os
import sys
import serial


# from firmata_testing import PyMata
# import statsmodels.formula.api as sm

# teensy boards require delay of 5 (arduino_wait)

class Experiment:
    def __init__(self, config_file):

        self.config_file = config_file
        self.configs = self.experiment_loader()
        print("config loaded!")

        # global variables
        self.tolerance = float(self.configs['tolerance'])  # how strict voltage keeping can be one after another
        # before tossing an error
        self.exptype = self.configs['experiment type']  # Morbidostat or Turbidostat
        self.resolution = float(self.configs['resolution'])  # how long in seconds the pump runs for every mix,
        # split between permissive and non-permissive
        self.initial_adaptation_cycles = int(self.configs['initial cycles'])  # burn in to get growth rate
        self.adaptation_cycles_to_step = int(self.configs['adaptation cycles'])  # # rounds that it has to be under
        # wt growth rate to step
        self.analog = self.configs['analog']  # i2c or direct from unit
        self.step_size = float(self.configs['step size'])  # how fine a step is
        self.reads_normal = int(self.configs['reads normal'])  # how many reads are done in one voltage read
        self.sleeptime_normal = float(self.configs['sleeptime normal'])  # how long you sleep between reads
        self.reads_diluting = int(self.configs['reads diluting'])  # ditto, but for diluting
        self.sleeptime_diluting = float(self.configs['sleeptime diluting'])  # ditto, but for diluting
        self.i2c_address_1 = int(self.configs['i2c board 1'])  # i2c board address for pins 1-16
        self.save_every = int(self.configs['save every'])  # how many voltage read rounds until a save
        self.growth_rate_type = self.configs['growth rate type']  # old or slope based
        self.interactive = self.configs['interactive mode']  # for use in ipython and troubleshooting
        self.name = self.configs['name']  #
        self.wiggle = float(self.configs['wiggle'])  # how close is good enough to WT growth rate, in seconds
        self.autovolt = self.configs['auto volt']
        self.verbose = self.configs['verbose']
        self.override_mix = self.configs['mix']
        # global file-based setup



        self.board_port = self.configs['board port']
        self.board = self.board_init()
        print("board initialized!")

        if self.analog == "FALSE":
            self.i2c_config()

        self.save_file = self.configs['save_file']
        self.new_experiment = self.configs['new experiment']
        self.Save = Save(self)
        self.sample_save = self.configs['sample save file']
        self.file_info = []
        self.size = 0

        try:
            self.file_info = os.stat(self.save_file)
        except IOError:
            print("io error - this is normal for a new experiment")
        try:
            self.size = self.file_info.st_size
        except AttributeError:
            self.size = False

        if self.new_experiment == 'TRUE' and self.size:
            try:
                print("Save file exists, are you sure you want to overwrite?")
                raise Exception
            except Exception:
                x = input('yes/no?')
                if x == 'yes':
                    pass
                else:
                    sys.exit()
        elif self.new_experiment == 'TRUE':
            self.Save.init_save_file(self.save_file)
            print("save file created!")
        else:
            self.Save.resume_from_save(self.save_file)
            print("save file loaded!")

        self.sample_file = self.configs['sample file']
        if self.new_experiment == 'TRUE':
            self.samples = self.sample_loader()
            self.sample_objects = []
            for sample in self.samples:
                self.sample_objects.append(self.sample_config(self.samples[sample]))
                print("sample %s loaded!" % self.samples[sample]['channel'])
            for sample in self.sample_objects:
                if self.override_mix == 'TRUE':
                    sample.mix_override()

        else:
            for sample in self.sample_objects:
                sample.experiment = self
                sample.sample_analog_init()
                # sample.sample_pump_init()
                sample.voltage_init()
                if self.override_mix == 'TRUE':
                    sample.mix_override()
                print("sample %s loaded" % sample.channel)
                sample.load_flag = 1

        print("all samples init!")

        # plotly init
        self.plotly_on = self.configs['plotly']
        if self.plotly_on == 'TRUE':
            self.plotly_maxpoints = self.configs['maxpoints']
            self.plotly_api_key = self.configs['api key']
            self.plotly_username = self.configs['plotly username']
            self.stream_id_list = []
            self.plotly_dict = {}
            self.trace_list = []
            for sample in self.sample_objects:
                plotly_stream = self.create_plotly_stream(sample.stream_id)
                self.stream_id_list.append(sample.stream_id)
                self.plotly_dict[sample] = plotly_stream
                self.trace_list.append(plotly_stream.trace)

            self.create_plotly(self.stream_id_list, self.trace_list)
            print("plotly online!")

        # i2c control


        self.timer = 0
        self.experiment_read_rounds = 0

        print("Experiment loaded!")
        if __name__ == '__main__':
            try:
                self.main(rounds=36000, exit_save="yes")
            except KeyboardInterrupt:
                print("keyboard int")
                with open(self.sample_save, 'wb') as file:
                    pickle.dump(self.sample_objects, file)

    def main(self, **kwargs):
        called_count = 0
        while __name__ == '__main__' or called_count < kwargs['rounds']:
            called_count += 1
            self.timer += 1
            self.experiment_read_rounds += 1
            for sample in self.sample_objects:
                sample.update_voltage_n()
            if self.timer > self.save_every:
                self.timer = 0
                print('Run saved! This was round %i' % self.experiment_read_rounds)
                for sample in self.sample_objects:
                    self.Save.update_save_file(self.save_file, sample)
                with open(self.sample_save, 'wb') as save_sample_file:
                    pickle.dump(self.sample_objects, save_sample_file, protocol=pickle.HIGHEST_PROTOCOL)
        print("main done!")
        try:
            if kwargs['exit_save'] == 'yes':
                with open(self.sample_save, 'wb') as file:
                    pickle.dump(self.sample_objects, file, protocol=pickle.HIGHEST_PROTOCOL)
        except KeyError:
            print("Exit Save failed!")

    def experiment_loader(self):
        config_dict = {}
        with open(self.config_file, newline='') as config_file_open:
            config_file_csv = csv.DictReader(config_file_open, dialect='excel')
            for line in config_file_csv:
                config_dict[line['name']] = line['value']
        return config_dict

    def sample_loader(self):
        sample_dict = {}
        with open(self.sample_file, newline='') as sample_file_open:
            sample_file_csv = csv.DictReader(sample_file_open, dialect='excel')
            for line in sample_file_csv:
                sample_dict[line['channel']] = line
        return sample_dict

    def board_init(self):
        try:
            board = PyMata(self.board_port, bluetooth=True)
            board.reset()
        except serial.serialutil.SerialException:
            print("Error, board not found, exiting")
            sys.exit()
        return board

    def i2c_config(self):
        # remember, you need pull UP resistors. Usually you need a small sleep between i2c commands to make it happy
        # todo: generalize i2c boards to n addresses
        self.board.i2c_config()
        time.sleep(1)
        self.board.i2c_read(self.i2c_address_1, 0x00, 2, self.board.I2C_READ)
        time.sleep(1)
        try:
            print(self.board.i2c_get_read_data(self.i2c_address_1))
        except TypeError:
            print("You probably forgot the pullup resistors on address %i" % self.i2c_address_1)
            raise

        # for mcp23017, this sets banks 0 and 1 to output mode instead of the default input
        self.board.i2c_write(self.i2c_address_1, 0x00, 0x00, 0x00)
        time.sleep(1)
        self.board.i2c_write(self.i2c_address_1, 0x01, 0x00, 0x00)
        time.sleep(1)
        self.board.i2c_write(self.i2c_address_1, 0x12, 0x00, 255)
        time.sleep(1)
        self.board.i2c_write(self.i2c_address_1, 0x13, 0x00, 255)
        time.sleep(1)
        print("all on")
        self.board.i2c_write(self.i2c_address_1, 0x12, 0x00, 0x00)
        self.board.i2c_write(self.i2c_address_1, 0x13, 0x00, 0x00)
        time.sleep(1)
        print("all off")

    def sample_config(self, sample):
        sample_object = Sample(sample['name'], sample['channel'], int(sample['analog_pin']), sample['pump_rest_pin'],
                               sample['pump_perm_pin'], sample['pump_waste_pin'], sample['v_adj'],
                               sample['growth_rate'], sample['volt_on'], sample['volt_off'], sample['stream_id'],
                               sample['experiment_number'], self)
        return sample_object

    def i2c(self, pin, state):
        if self.analog == 'TRUE':
            raise RuntimeError("i2c not init/ not valid, analog")
        # pins are 1-8 on bank 1 (eg from 00000001 to 10000000), 9-16 on bank 2
        # This is only for mcp23017 chipsets, your mileage may vary
        bank = 0

        if pin < 9:
            bank = 0
            self.board.i2c_read(self.i2c_address_1, 0x12, 2, self.board.I2C_READ)
        elif pin < 17:
            bank = 1
            self.board.i2c_read(self.i2c_address_1, 0x13, 2, self.board.I2C_READ)
        else:
            print("OHES NOES INVALID PIN NUMBER D:D:D:D:")
        pin -= 1
        time.sleep(1)
        curr_bank_state = self.board.i2c_get_read_data(self.i2c_address_1)
        print(curr_bank_state[1])
        pin %= 8
        if state == 1:
            # this should turn on only the pin in question and leave the rest as is. bank 0 = 0x12 eg 18,
            # bank 1 = 0x13 eg 19
            mask = 1 << pin
            self.board.i2c_write(self.i2c_address_1, 0x12 + bank, 0x00, curr_bank_state[1] | mask)
        elif state == 0:
            # same as above, but off
            mask = ~(1 << pin)
            self.board.i2c_write(self.i2c_address_1, 0x12 + bank, 0x00, curr_bank_state[1] & mask)
        else:
            print("i2c state error! you asked me to change to an invalid state!!!!")

    def create_plotly(self, stream_id_list, trace_list):
        # goes after plotly streams are initialized
        tls.set_credentials_file(username=self.plotly_username, api_key=self.plotly_api_key)
        tls.set_credentials_file(stream_ids=stream_id_list)

        data = Data(trace_list)
        layout = Layout(title="growth",
                        xaxis=XAxis(autorange=True), yaxis=YAxis(autorange=True))
        fig = Figure(data=data, layout=layout)
        unique_url = py.plot(fig, filename="growthstream_mgn", auto_open=False)

    def create_plotly_stream(self, stream_id):
        return PlotlyStream(stream_id, self)


class Save:
    def __init__(self, experiment):
        self.experiment = experiment
        self.unique_id = 1

    def init_save_file(self, savefile):
        with open(savefile, 'w') as csvfile:
            fieldnames = ['unique_id', 'sample', 'channel', 'experiment', 'last_volt', "volt_on", 'volt_off',
                          'v_adjustment',
                          'growth rate', 'currmix', 'rounds']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

    def resume_from_save(self, savefile):
        # append save file:
        with open(savefile, 'a') as csvfile:
            fieldnames = ['unique_id', 'sample', 'channel', 'experiment', 'last_volt', "volt_on", 'volt_off',
                          'v_adjustment',
                          'growth_rate', 'currmix', 'rounds']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            writer.writerow({'unique_id': self.unique_id, 'experiment': self.experiment.name,
                             'rounds': "experiment resumed resumed"})
        with open(self.experiment.sample_save, 'rb') as sample_save:
            self.experiment.sample_objects = pickle.load(sample_save)

    def update_save_file(self, savefile, sample_object):
        with open(savefile, 'a+') as csvfile:
            fieldnames = ['unique_id', 'sample', 'channel', 'experiment', 'last_volt', "volt_on", 'volt_off',
                          'v_adjustment',
                          'growth_rate', 'currmix', 'rounds']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writerow({'unique_id': self.unique_id, 'sample': sample_object.name,
                             'experiment': sample_object.experiment_number, 'last_volt': sample_object.last_volt,
                             'volt_on': sample_object.volt_on, "volt_off": sample_object.volt_off,
                             'v_adjustment': sample_object.v_adj, 'growth_rate': sample_object.growth_rate,
                             "currmix": sample_object.currmix,
                             'rounds': sample_object.experiment.experiment_read_rounds,
                             'channel': sample_object.channel})


class PlotlyStream:
    def __init__(self, stream_id, experiment):
        self.stream_id = stream_id
        self.experiment = experiment
        self.stream = Stream(token=self.stream_id, maxpoints=self.experiment.plotly_maxpoints)
        self.trace = Scatter(x=[], y=[], mode='lines+markers', stream=self.stream)
        self.Stream = py.Stream(self.stream_id)

    def update_plot(self, yval):
        try:
            self.Stream.open()
            xval = datetime.datetime.now()
            self.Stream.write(dict(x=xval, y=yval))
        except:
            print("skipping plotly, exception")


class Pump:
    def __init__(self, pin, channel, state, sample):
        self.pin = int(pin)
        self.channel = channel
        self.state = state
        self.sample = sample

        self.sample.experiment.board.set_pin_mode(self.pin, self.sample.experiment.board.OUTPUT,
                                                  self.sample.experiment.board.DIGITAL)

        if state == 'on':
            self.pump_on()

        elif state == 'off':
            self.pump_off()

        else:
            print("Error, invalid pump state on pump %s" % self.channel)

    def pump_on(self):
        if self.sample.experiment.analog == 'TRUE':
            self.sample.experiment.board.digital_write(self.pin, 1)
        else:
            self.sample.experiment.i2c(self.pin, 1)

    def pump_off(self):
        if self.sample.experiment.analog == "TRUE":
            self.sample.experiment.board.digital_write(self.pin, 0)
        else:
            self.sample.experiment.i2c(self.pin, 0)


class Sample:
    def __init__(self, name, channel, analog_pin, pump_rest_pin, pump_perm_pin, pump_waste_pin, v_adj, growth_rate,
                 volt_on, volt_off, stream_id, experiment_number, experiment):
        self.error_flag = 0
        self.name = name
        self.load_flag = 0
        self.channel = int(channel)
        self.analog_pin = int(analog_pin)
        self.pump_rest_pin = int(pump_rest_pin)
        self.pump_perm_pin = int(pump_perm_pin)
        self.pump_waste_pin = int(pump_waste_pin)
        self.pump_list = [self.pump_perm_pin, self.pump_rest_pin, self.pump_waste_pin]
        self.pump_obj = []
        self.v_adj = float(v_adj)
        self.growth_rate = float(growth_rate)
        self.volt_on = float(volt_on)
        self.volt_off = float(volt_off)
        self.stream_id = stream_id
        self.experiment_number = experiment_number
        self.experiment = experiment

        self.last_volt = float(0)
        self.curr_volt = float(0)
        self.volt_hist = []
        self.last_dilute_time = time.time()
        self.currmix = [1, 0]
        self.growth_rate_history = []
        self.volt_hist_this_dilute = {'Time': [], 'Voltage': []}

        self.wt_growth_rate = float(growth_rate)  # THIS IS OBVIOUSLY AN ERROR
        self.growth_counter = 0
        self.delta_voltage = []

        self.sample_analog_init()
        self.sample_pump_init()

        self.voltage_init()

        self.NEW_LOAD = 1

        self.time_init = time.time()

    def __getstate__(self):
        odict = self.__dict__.copy()
        del odict['experiment']
        return odict

    def __setstate__(self, state):
        print(state)
        self.__dict__ = state
        # self.sample_pump_init()
        # self.voltage_init()

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

    def sample_analog_init(self):
        self.experiment.board.set_pin_mode(int(self.analog_pin), self.experiment.board.INPUT,
                                           self.experiment.board.ANALOG)

    def sample_pump_init(self):
        for pump in self.pump_list:
            self.pump_obj.append(Pump(pump, self.channel, 'off', self))

    def voltage_init(self):
        self.voltage_handler = VoltageHandler(self)
        print("Channel %i variables analog %i pumps %s volts on %i off %i streamid %s" % (self.channel, self.analog_pin,
                                                                                          self.pump_list, self.volt_on,
                                                                                          self.volt_off, self.stream_id)
              )
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
        self.last_volt = self.voltage_handler.voltage_average_d()
        print(self.last_volt)

    def update_voltage_n(self):
        self.curr_volt = float(self.voltage_handler.voltage_average_n())
        print(u'Voltage on channel {0:d} is {1:f}'.format(self.channel, self.curr_volt))
        if not self.load_flag == 1:
            if self.error_flag == 1:
                print(u"channel {0:d} is in error state".format(self.channel))
                return

        if self.experiment.tolerance < abs(float(self.last_volt) - float(self.curr_volt)):
            if self.load_flag == 1:
                self.load_flag = 0
                if self.error_flag == 1:
                    self.error_flag = 0
            else:
                pre_spike = self.last_volt

                while (self.experiment.tolerance / 4) < abs(float(self.last_volt) - float(self.curr_volt)):
                    self.update_voltage_history(self.curr_volt)
                    print(
                        u"Voltage spike on channel {0:d}, is now {1:f}, was {2:f}".format(self.channel, self.curr_volt,
                                                                                          self.last_volt))
                    time.sleep(1)
                    self.last_volt = self.curr_volt
                    self.curr_volt = self.voltage_handler.voltage_average_n()
                self.v_adj += (self.curr_volt - pre_spike)
            self.last_volt = self.voltage_handler.voltage_average_n()

        elif float(self.curr_volt) < float(self.volt_on) and not (
                self.NEW_LOAD == 1 and self.experiment.autovolt == "TRUE"):
            if self.load_flag == 1:
                self.load_flag = 0
                if self.error_flag == 1:
                    self.error_flag = 0
            self.update_voltage_history(self.curr_volt)
            self.volt_hist_this_dilute["Time"].append(time.time())
            self.volt_hist_this_dilute['Voltage'].append(self.curr_volt)
            time.sleep(.05)
            self.curr_volt = self.voltage_handler.voltage_average_n()
            if self.curr_volt < float(self.volt_on):

                if self.experiment.exptype == "Turbidostat":
                    self.t_dilute()
                    if self.NEW_LOAD == 1:
                        if self.experiment.verbose == "TRUE":
                            print("turning off newload flag in t.dilute")
                        self.NEW_LOAD = 0
                        self.last_dilute_time = time.time()

                elif self.experiment.exptype == 'Morbidostat':
                    if self.NEW_LOAD == 1:
                        if self.experiment.verbose == "TRUE":
                            print("turning off newload flag in m.dilute")
                        self.NEW_LOAD = 0
                        self.last_dilute_time = time.time()
                    self.m_dilute()
                    self.delta_voltage = []

                else:
                    print("error, shouldn't be here")
                    self.last_volt = self.curr_volt
        else:
            if self.load_flag == 1:
                if self.experiment.verbose == "TRUE":
                    print("turning off load flag due to normal")
                self.load_flag = 0
            self.delta_voltage.append(self.last_volt - self.curr_volt)
            self.last_volt = self.curr_volt
            self.update_voltage_history(self.curr_volt)

        if self.experiment.plotly_on == "TRUE":
            self.experiment.plotly_dict[self].update_plot(self.curr_volt)

        if self.NEW_LOAD == 1 and self.experiment.autovolt == "TRUE":
            if self.experiment.verbose == "TRUE":
                print("volt start %f" % self.volt_hist[0])

            if len(self.volt_hist) < 51:
                pass
            else:
                differences = 0.0
                list1 = []
                for y in self.volt_hist[len(self.volt_hist) - 50:]:
                    list1.append(y)

                for i, y in enumerate(list1):
                    if i == 0:
                        pass
                    else:
                        differences += (list1[i] - list1[i - 1])
                differences = -differences
                if self.experiment.verbose == "TRUE":
                    print("current delta is %f " % differences)
                if self.curr_volt < (self.volt_hist[0] - .5) and differences < .075 and (
                    time.time() - self.time_init) > 86400:
                    if self.experiment.verbose == "TRUE":
                        print("turning off new_load flag in autovolter")
                    self.NEW_LOAD = 0
                    volt_threshold = self.auto_volter(self.volt_hist)
                    self.volt_on = volt_threshold[0]
                    self.volt_off = volt_threshold[1]
                    self.last_dilute_time = time.time()

                    # todo: plotly integration
                    # autovolter assignment is backwards blooooop

    def auto_volter(self, volt_history):
        FIRST_VOLTS = 10
        LAST_VOLTS = 10
        DELTA = .10  # (percentage of total range that offsets volt_on, volt_off)
        sum_first = 0
        sum_last = 0
        for i in range(0, FIRST_VOLTS):
            sum_first += volt_history[i]
        sum_first /= FIRST_VOLTS

        for i in range(0, LAST_VOLTS):
            sum_last += volt_history[len(volt_history) - LAST_VOLTS + i]
        sum_last /= LAST_VOLTS
        range_voltage = sum_first - sum_last
        midpoint = (sum_first + sum_last) / 2

        return_voltages = [0, 0]

        return_voltages[0] = midpoint - (range_voltage * DELTA)  # volt_on
        return_voltages[1] = midpoint + (range_voltage * DELTA)  # volt_off
        if self.experiment.verbose == "TRUE":
            print(return_voltages)
            print("return voltages are: in autovolter")
        return return_voltages

    def update_growth_rate_regression(self):
        v_hist_regress = self.volt_hist_this_dilute
        data = pd.DataFrame.from_dict(v_hist_regress)
        regression = sm.ols('Voltage ~ Time', data=data).fit()
        params = regression.params
        self.growth_rate_history.append(params[0])

    def update_voltage_history(self, voltage):
        if len(self.volt_hist) < 3600:
            self.volt_hist.append(voltage)
        else:
            self.volt_hist.append(voltage)
            self.volt_hist.pop(0)

    def update_growth_rate(self, rate):
        # Growth_rate list holds a history of all growth rates, and compares it to WT growth rate. If it's faster than
        # WT for some period of time, increment the morbidostat (see below). Remember, lower is faster!
        self.growth_rate = rate
        print("updating growth rate")
        if len(self.growth_rate_history) < self.experiment.initial_adaptation_cycles:
            try:
                if self.growth_rate < 3600:  # sanity check
                    pass
                    print("growth rate less than 1 hr, passing")
                elif self.growth_rate < min(self.growth_rate_history):
                    self.wt_growth_rate = self.growth_rate
                    self.growth_rate_history.append(rate)
                    print("faster than min before, appending")
                else:
                    self.growth_rate_history.append(rate)
                    print('yay appending in min growth rate')
            except ValueError:
                print("value error")
                self.wt_growth_rate = self.growth_rate
                self.growth_rate_history.append(rate)

                # todo fix this add in punt on v low rfgowth rate
        elif len(self.growth_rate_history) < 5000:
            self.growth_rate_history.append(rate)
        else:
            self.growth_rate_history.append(rate)
            self.growth_rate_history.pop(0)

    def t_dilute(self):
        # turbidostat dilutions. Dilute with permissive until voltage is above a certain cutoff
        rate_growth = time.time() - self.last_dilute_time
        if self.experiment.growth_rate_type == 'old':
            self.update_growth_rate(rate_growth)
        elif self.experiment.growth_rate_type == 'regression':
            self.update_growth_rate_regression()
        self.last_dilute_time = time.time()
        timer = 0
        while float(self.voltage_handler.voltage_average_d()) < float(self.volt_off):
            timer += 1
            self.pump_obj[0].pump_on()
            self.pump_obj[2].pump_on()
            time.sleep(.1)
            print("Channel %i is diluting" % self.channel)
            self.pump_obj[0].pump_off()
            time.sleep(5)
            self.pump_obj[2].pump_off()
            time.sleep(.1)
            print(self.voltage_handler.voltage_average_d())
            if timer > 25:
                self.error_flag = 1
                break

        self.pump_obj[0].pump_off()
        time.sleep(10)  # this is important to allow for differences in pump speed to wash out. Don't want to accidently
        # overflow
        self.pump_obj[2].pump_off()
        self.last_volt = self.voltage_handler.voltage_average_d()
        self.volt_hist_this_dilute = {'Time': [], 'Voltage': []}

    def pump_mixer(self):
        # This mixes permissive and non-permissive together. Might not work with small values of resolution
        permissive_duty = self.currmix[0]
        if permissive_duty < 0:
            permissive_duty = 0  # catches invalid duty numbers by bounding to 0 for permissive and 1 for nonpermissive,
            # respectively
        nonpermissive_duty = self.currmix[1]
        if nonpermissive_duty > 1:
            nonpermissive_duty = 1
        print("diluting permissive")
        self.pump_obj[0].pump_on()
        time.sleep(permissive_duty * self.experiment.resolution)
        self.pump_obj[0].pump_off()
        print("diluting non-permissive")
        self.pump_obj[1].pump_on()
        time.sleep(nonpermissive_duty * self.experiment.resolution)
        self.pump_obj[1].pump_off()

    def m_mixer_check(self):
        # if we're in the initial cycles, don't increment the counter that steps non-permissive up
        if len(self.growth_rate_history) < self.experiment.initial_adaptation_cycles:
            print("passing due to initial growth cycles")
            pass
        else:
            # otherwise, increment the counter if we're growing faster than WT. If not, reset counter to 0

            if self.growth_rate <= (self.wt_growth_rate + self.experiment.wiggle):
                self.growth_counter += 1
                print("stepping growth counter")
            elif self.growth_rate > (self.wt_growth_rate + self.experiment.wiggle):
                self.growth_counter = 0
                print("resetting growth counter")
            if self.growth_counter >= self.experiment.adaptation_cycles_to_step:
                # if counter is high enough, step permissive and nonpermissive down and up, and reset counter
                self.growth_counter = 0
                print("stepping mix")
                if self.currmix[0] > 0:
                    self.currmix[0] -= self.experiment.step_size
                    self.currmix[1] += self.experiment.step_size
                else:
                    pass

        if self.experiment.verbose == "TRUE":
            print(self.growth_counter)
            print(self.growth_rate)
            print(str(float(self.wt_growth_rate) + float(self.experiment.wiggle)))

    def m_dilute(self):

        rate_growth = time.time() - self.last_dilute_time
        if self.experiment.growth_rate_type == 'old':
            self.update_growth_rate(rate_growth)
        elif self.experiment.growth_rate_type == 'regression':
            self.update_growth_rate_regression()
        self.last_dilute_time = time.time()  # update growth rate list and reset last dilute time
        self.m_mixer_check()  # check to see if we're due for a step
        timer = 0
        while float(self.voltage_handler.voltage_average_d()) < float(self.volt_off):
            timer += 1
            # and until back above volt_off, dilute using pump_mixer
            print(self.voltage_handler.voltage_average_d())
            self.pump_obj[2].pump_on()
            self.pump_mixer()
            print("Channel %i is diluting at mix %f %f" % (self.channel, self.currmix[0], self.currmix[1]))
            time.sleep(10)
            self.pump_obj[2].pump_off()
            time.sleep(.1)
            if timer > 25:
                self.error_flag = 1
                break

        self.last_volt = self.voltage_handler.voltage_average_d()
        self.volt_hist_this_dilute.clear()
        self.volt_hist_this_dilute = {'Time': [], 'Voltage': []}


class VoltageHandler:
    def __init__(self, sample):
        self.analog_pin = sample.analog_pin
        self.sample = sample

    def voltage_average_n(self):
        avgread = 0.0

        for i in range(0, self.sample.experiment.reads_normal):
            avgread += self.sample.experiment.board.analog_read(self.analog_pin) / 1024.0 * 5.0
            time.sleep(self.sample.experiment.sleeptime_normal)
        avgread = (avgread / self.sample.experiment.reads_normal) - self.sample.v_adj
        assert isinstance(avgread, float)
        return float(avgread)

    def voltage_average_d(self):
        avgread = 0.0

        for i in range(0, self.sample.experiment.reads_diluting):
            avgread += self.sample.experiment.board.analog_read(self.analog_pin) / 1024.0 * 5.0
            time.sleep(self.sample.experiment.sleeptime_diluting)
        avgread = (avgread / self.sample.experiment.reads_diluting) - self.sample.v_adj
        assert isinstance(avgread, float)
        return float(avgread)


if __name__ == '__main__':
    experiment = Experiment('/Users/mnapolitano/Dropbox/Morbidostat/v3_test/Experiment_file.csv')
