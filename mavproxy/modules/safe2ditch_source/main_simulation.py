# -*- coding: utf-8 -*-

import argparse
from config_file_parser import ConfigFileParser
from simulation_tools.kmz_writer import KmzWriter
from simulation_tools.simulation_exec import SimulationExec 
from simulation_tools.run_simulation import RunSimulation

class CommandLineParser:
    
    def determine_config_info(self):
        parser = argparse.ArgumentParser(description='Safe2Ditch Simulator')
        parser.add_argument('-f', '--file',
                            type=str,
                            required=False,
                            default='config_file.json',
                            help='desired config file.')
        parser.add_argument('-t', '--tolerance',
                            type=float,
                            required=False,
                            default=0,
                            help='distance tolerance for vehicle in ft.')
        parser.add_argument('-e', '--engage_time',
                            type=float,
                            required=False,
                            default=60,
                            help='time for the vehicle to engage Safe2Ditch.')
        parser.add_argument('-s1', '--stage_1',
                            required=False,
                            action='store_true',
                            help='stage 1 of the simulation will be selected.')
        parser.add_argument('-s2', '--stage_2',
                            required=False,
                            action='store_true',
                            help='stage 2 of the simulation will be selected.')
        parser.add_argument('-ct', '--crash_time',
                            type=float,
                            required=False,
                            default=30,
                            help='time the vehicle has before crash (in seconds).')
        
        return parser.parse_args()
        
        
class MainSimulation:
    
    def __init__(self, parser_args):
        self.parser = ConfigFileParser(parser_args.file)
        self.animation_file_name = 'animation.kmz'
        self.engage_time = parser_args.engage_time
        self.tolerance = parser_args.tolerance
        self.se = SimulationExec(self.parser.data)
    
    def determine_animation_name(self, argv):
        config_main = str(argv).split(".")
        # Append the config file name to the animation file to colate
        self.animation_file_name = "animation_" + config_main[0] + ".kmz"
        
    def setup_simulation_stage1(self, config_file):
        self.determine_animation_name(config_file)
        run_simulation = RunSimulation(self.engage_time,
                                       self.parser.data,
                                       self.parser.route_file_package,
                                       self.parser.ditch_site_package,
                                       self.parser.veh_path_file,
                                       self.tolerance,
                                       "MyCopter/ditch_sites.txt")
        run_simulation.run_stage1()
        run_writer = KmzWriter(self.parser.veh_path_file, 
                               self.parser.route_file_package,
                               self.parser.ditch_site_package,
                               self.animation_file_name)
        run_writer.run_no_engage()                                         
        
    def setup_simulation_stage2(self, config_file, time_to_crash):
        self.determine_animation_name(config_file)
        run_simulation = RunSimulation(self.engage_time,
                                       self.parser.data,
                                       self.parser.route_file_package,
                                       self.parser.ditch_site_package,
                                       self.parser.veh_path_file,
                                       self.tolerance,
                                       "MyCopter/ditch_sites.txt")
        run_simulation.run_stage2(time_to_crash)
        run_writer = KmzWriter(self.parser.veh_path_file, 
                               self.parser.route_file_package,
                               self.parser.ditch_site_package,
                               self.animation_file_name)
        run_writer.run(run_simulation.list_of_big_Xs)                                                      
        
    def run(self, argv, stage_2, time_to_crash):
        if stage_2 == True:
            self.setup_simulation_stage2(argv, time_to_crash)
            
        else:
            self.setup_simulation_stage1(argv)
    
            
if __name__ == '__main__':
   
    CommandLineParser = CommandLineParser()
    parser = CommandLineParser.determine_config_info()
    main_sim = MainSimulation(parser) 
    main_sim.run(parser.file, parser.stage_2, parser.crash_time)
    



