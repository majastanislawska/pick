# Support for logging periodic statistics
#
# Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import psutil, time, logging

class PrinterSysStats:
    def __init__(self, config):
        printer = config.get_printer()
        self.last_process_time = self.total_process_time = 0.
        self.last_load_avg = 0.
        self.last_mem_avail = 0

    def stats(self, eventtime):
        # Get core usage stats
        ptime = time.process_time()
        pdiff = ptime - self.last_process_time
        self.last_process_time = ptime
        if pdiff > 0.:
            self.total_process_time += pdiff
        self.last_load_avg = psutil.getloadavg()[0]
        svmem = psutil.virtual_memory()
        self.last_mem_avail = svmem.available
        msg = "sysload=%.2f cputime=%.3f memavail=%d" % (self.last_load_avg,
                                             self.total_process_time,
                                             self.last_mem_avail)
        return (False, msg)
    def get_status(self, eventtime):
        return {'sysload': self.last_load_avg,
                'cputime': self.total_process_time,
                'memavail': self.last_mem_avail}

class PrinterStats:
    def __init__(self, config):
        self.printer = config.get_printer()
        reactor = self.printer.get_reactor()
        self.stats_timer = reactor.register_timer(self.generate_stats)
        self.stats_cb = []
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
    def handle_ready(self):
        self.stats_cb = [o.stats for n, o in self.printer.lookup_objects()
                         if hasattr(o, 'stats')]
        if self.printer.get_start_args().get('debugoutput') is None:
            reactor = self.printer.get_reactor()
            reactor.update_timer(self.stats_timer, reactor.NOW)
    def generate_stats(self, eventtime):
        stats = [cb(eventtime) for cb in self.stats_cb]
        if max([s[0] for s in stats]):
            stats_str = ' '.join([s[1] for s in stats if s[1]])
            logging.info("Stats %.1f: %s", eventtime, stats_str)
        return eventtime + 1.

def load_config(config):
    config.get_printer().add_object('system_stats', PrinterSysStats(config))
    return PrinterStats(config)
