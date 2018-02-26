import os
import math
from waypoint import Waypoint
from visionDataReceiver import VisionDataReceiver

class TriageLogic:

    def __init__(self, ditch_site_data, output_file):
        self.ditch_site_data = ditch_site_data

        print "TL: ditch sites available:"
        for i in range(len(self.ditch_site_data)):
            print self.ditch_site_data[i].name + ","
        print "\n"

        # Rescale dictionary values for ditch site as needed
        feetToMeters = 0.3048
        for line in range(len(self.ditch_site_data)):
            self.ditch_site_data[line].alt = self.ditch_site_data[line].alt * feetToMeters
            #self.ditch_site_data[line].reliability = self.ditch_site_data[line].reliability * 0.01 # Scale percentage to decimal

        self.file = self.open_file(output_file)

        # This needs to come from the parser, but set here for now
        # Flag indicates if intruder data will be coming in from an outside source
        self.intruders = True

        if (self.intruders):
            self.visionReceiver = VisionDataReceiver()
            self.visionReceiver.run()

    @staticmethod
    def open_file(output_file):
        
        # returns the file object so that it can be closed later
        if os.path.isfile(output_file):
            try:
                file = open(output_file, 'w')
            except IOError as e:
                pass
        else:
            file = open(output_file, 'a')
                
        return file

    # returns the memory location of a variable
    def memory_address(self, in_var):
        return hex(id(in_var))

    def limit(self, num_eval, minAllowed, maxAllowed):
        
        return max(minAllowed, min(num_eval, maxAllowed))

    #def select_site(self, timeLeftSec, vehSpeedFtPerSec, veh_wp: Waypoint):
    def select_site(self, timeLeftSec, vehSpeedFtPerSec, veh_wp):
        listOfBigXs = []
        rangeTolerance = 0.8 
        vehicleRange = 500 
        bufferedMaxRange = 500
        minBestRadius = 20 
        maxBestRadius = 60 # no extra weight for radius beyond this
        radiusWeightFactor = 0.2
        reliabilityWeightFactor = 3.0
        intruderWeightFactor = -100.0
        # lat/lon estimator site: http://www.csgnetwork.com/degreelenllavcalc.html)
        latFtPerDegLangley = 364107.19 
        lonFtPerDegLangley = 291526.76 
        
        # VehicleRange is how far the vehicle thinks is its max range at engagement
        vehicleRange = timeLeftSec * vehSpeedFtPerSec
        # bufferedMaxRange is a smaller distance to add margin
        # rangeDelta is the difference in feet, for convenience in rangeFactor calc
        bufferedMaxRange = vehicleRange * rangeTolerance
        rangeDelta = vehicleRange - bufferedMaxRange

        # Add the vehicle lat/lon location to the list which will be passed to the KmzWriter
        # to place an X on the animation for where vehicle was when engagement occurred 
        listOfBigXs.append(veh_wp)

        # Using squared values here to save computation speed by not having to take 
        # square root until last minute
        vehicleRangeSquared = vehicleRange**2

        # Go through full file of regional ditch sites, and determine which are 
        # in range of crippled vehicle
        usableSites = []
        radiansToDeg = 180.0/3.14159
        for line in range(len(self.ditch_site_data)):
            #centerLat = self.ditch_site_data[line].lat
            #centerLon = self.ditch_site_data[line].lon
            
            centerLatRad = self.ditch_site_data[line].lat
            centerLonRad = self.ditch_site_data[line].lon
            centerLat = centerLatRad * radiansToDeg
            centerLon = centerLonRad * radiansToDeg
            vehLatDeg = veh_wp.lat * radiansToDeg
            vehLonDeg = veh_wp.lon * radiansToDeg
            
            siteRangeX = (vehLatDeg - centerLat) * latFtPerDegLangley
            siteRangeY = (vehLonDeg - centerLon) * lonFtPerDegLangley
            
            siteRangeSquared = siteRangeX * siteRangeX + siteRangeY * siteRangeY

            siteRange = math.sqrt(siteRangeSquared)

            print "TL: veh rng= " + str(vehicleRange) + ", rng to " + self.ditch_site_data[line].name + " is " + str(siteRange)
            
            if (siteRangeSquared <= vehicleRangeSquared):
                rangeToSite = math.sqrt(siteRangeSquared)
                self.ditch_site_data[line].range = rangeToSite
                usableSites.append(self.ditch_site_data[line])
                print "usable sites includes " + self.ditch_site_data[line].name

        # Parse through subset of usable sites, and compute rangeFactor, radiusFactor, and desirability

        if len(usableSites) > 0:
            # pick the "best" site as first value, update it continuously as new best site is found
            bestSite = usableSites[0]

            for siteOption in range(len(usableSites)):
               # compute range factor and update for this siteOption
               rangeFactor = 0.0
               if usableSites[siteOption].range < bufferedMaxRange:
                   # Within first region of range function the slope allows a 20% (0.2)
                   # decrease through the bufferedMaxRange
                   maxValue = 1.0
                   slope = -0.2
                   rangeFactor = (maxValue 
                                 + (usableSites[siteOption].range 
                                    / bufferedMaxRange) 
                                 * slope)
               elif usableSites[siteOption].range <  vehicleRange:
                   # Within second (steeper slope)region of range function. 80% (0.8)
                   # slope
                   maxValue = 0.8
                   slope = -0.8
                   rangeFactor = (maxValue 
                                  + ((usableSites[siteOption].range 
                                      - bufferedMaxRange)
                                     / (vehicleRange - bufferedMaxRange)) 
                                  * slope)

               usableSites[siteOption].range_factor = rangeFactor

               # compute radius factor and update for this siteOption
               usableSites[siteOption].radius_factor = (self.limit((usableSites[siteOption].radius 
                                                                   - minBestRadius), 0., 
                                                                  maxBestRadius) 
                                                                 / (maxBestRadius 
                                                                    - minBestRadius))

               # if using intruder data, update the intrude factor to be the number of 
               # intruders within site. Defaults to 0 if no intruder data in use.
               if (self.intruders):
                   usableSites[siteOption].intruder_factor = self.visionReceiver.getNumberIntrudersWithin(usableSites[siteOption])
                   if (usableSites[siteOption].intruder_factor > 0):
                       print usableSites[siteOption].name + " intuders:" + str(usableSites[siteOption].intruder_factor),
                       print ", desirability: " + str(usableSites[siteOption].desirability)

               # compute desirability and update for this siteOption
               usableSites[siteOption].desirability = (usableSites[siteOption].range_factor
                                                       + usableSites[siteOption].radius_factor
                                                       * radiusWeightFactor
                                                       + usableSites[siteOption].intruder_factor
                                                       * intruderWeightFactor
                                                       + usableSites[siteOption].reliability
                                                       * reliabilityWeightFactor)

               # compare this site's desirability, and save as best site if it exceeds
               # the desirability of previous best site

               if usableSites[siteOption].desirability > bestSite.desirability:
                   bestSite = usableSites[siteOption]
           
            # Add this vehicle lat/lon location to the list which will be passed to the KmzWriter
            # to place an X on the animation for where vehicle was when engagement occurred 
            listOfBigXs.append(bestSite)

            "******************** For Debugging ******************************"
            """print("Selected ditch site: ", bestSite.name) 
               print("Desirability: ", bestSite.desirability)"""
            "*****************************************************************"
            print("Selected ditch site: ", bestSite.name),
            print("Desirability: ", bestSite.desirability)
            

        # List will have at least one big X for where vehicle was located, 
        # and two if bestSite was found
        return listOfBigXs
    
    #def get_best_site(self, time_left_sec, veh_speed, veh_wp: Waypoint):
    def get_best_site(self, time_left_sec, veh_speed, veh_wp):
        """ This function returns one waypoint from select_site 
        for the simulator """    
        site_selected = self.select_site(time_left_sec, veh_speed, veh_wp)
        best_site = Waypoint(veh_wp.lat, veh_wp.lon, veh_wp.alt)
        
        if len(site_selected) == 2:
            best_site = Waypoint(site_selected[1].lat, 
                                 site_selected[1].lon, 
                                 site_selected[1].alt)
        
        return best_site
    
    #def select_new_waypoint(self, time_left_sec, veh_speed, veh_wp: Waypoint):
    def select_new_waypoint(self, time_left_sec, veh_speed, veh_wp):
    
        """Uses distance from desired point to the selected ditch site
        (45 45 90), which means distance is equal to altitude. Uses bearing 
        from current lat, lon to ditch site lat, lon"""
        
        new_path_options = []
        best_site = self.get_best_site(time_left_sec, veh_speed, veh_wp)
        distance = veh_wp.alt
            
        bearing = best_site.bearing(veh_wp)
        new_wp_lat = best_site.calc_new_lat(bearing, distance)
        new_wp_lon = best_site.calc_new_lon(new_wp_lat, bearing, distance)
        
        tod_wp_name = best_site.name + "TOD"
        new_waypoint_option = Waypoint(new_wp_lat, new_wp_lon, veh_wp.alt, tod_wp_name)
        
        new_path_options.append(new_waypoint_option)
        new_path_options.append(best_site)
        
        "******************** For Debugging ******************************"
        """print('Point of Descent:', math.degrees(new_waypoint_option.lat), math.degrees(new_waypoint_option.lon), new_waypoint_option.alt)
           print('Ditch site:', math.degrees(best_site.lat), math.degrees(best_site.lon), best_site.alt)"""
        "*****************************************************************"
        
        return new_path_options
    
    #def output_best_site_for_ralley(self, time_left_sec, veh_speed, veh_wp: Waypoint):
    def output_best_site_for_ralley(self, time_left_sec, veh_speed, veh_wp):
        
        best_site = self.get_best_site(time_left_sec, veh_speed, veh_wp)
        self.file.write("{:0.6f} {:1.6f} {:2.6f}".format(best_site.lat,
                                                         best_site.lon, 
                                                         best_site.alt))
        
    

