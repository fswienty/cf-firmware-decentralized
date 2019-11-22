import time

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

class HelperFunctions:

##### PARAMETER GET AND SET METHODS #####
    #param = 0
    param_name = ""
    uri = ""

    def set_param(self, scf, param_name, value):
        cf = scf.cf
        cf.param.set_value(param_name, value)


    def get_param(self, scf, param_name):
        self.param_name = param_name
        self.uri = scf._link_uri
        log_conf = LogConfig(name=self.param_name, period_in_ms=75)
        log_conf.add_variable(self.param_name, 'float')
        scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self.get_param_callback)
        log_conf.start()
        time.sleep(0.1)
        log_conf.stop()
        #return self.param


    def get_param_callback(self, timestamp, data, logconf):
        value = data[self.param_name]
        #self.param = value
        #print(value)
        print(f"{self.uri}: {data}")


##### ESTIMATOR RESET AND STARTUP STUFF #####
    def wait_for_position_estimator(self, scf):
        print('Waiting for estimator to find position...')

        log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
        log_config.add_variable('kalman.varPX', 'float')
        log_config.add_variable('kalman.varPY', 'float')
        log_config.add_variable('kalman.varPZ', 'float')

        var_y_history = [1000] * 10
        var_x_history = [1000] * 10
        var_z_history = [1000] * 10

        threshold = 0.001

        with SyncLogger(scf, log_config) as logger:
            for log_entry in logger:
                data = log_entry[1]

                var_x_history.append(data['kalman.varPX'])
                var_x_history.pop(0)
                var_y_history.append(data['kalman.varPY'])
                var_y_history.pop(0)
                var_z_history.append(data['kalman.varPZ'])
                var_z_history.pop(0)

                min_x = min(var_x_history)
                max_x = max(var_x_history)
                min_y = min(var_y_history)
                max_y = max(var_y_history)
                min_z = min(var_z_history)
                max_z = max(var_z_history)

                # print("{} {} {}".
                #       format(max_x - min_x, max_y - min_y, max_z - min_z))

                if (max_x - min_x) < threshold and (
                        max_y - min_y) < threshold and (
                        max_z - min_z) < threshold:
                    break


    def reset_estimator(self, scf):
        cf = scf.cf
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        self.wait_for_position_estimator(cf)
    
    
    def wait_for_param_download(self, scf):
        while not scf.cf.param.is_updated:
            time.sleep(1.0)
        print('Parameters downloaded for', scf.cf.link_uri)