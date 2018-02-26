import math 

# This class and program create kmz format using a route file and a ditch sites
# file as input. It prints to screen, and output must be captured to output file 
# of choice. Usage:
#
# python KmzWriter.py >my_kmz_file.kmz

from simulation_tools.intruder import Intruder
 
class KmzWriter:
    def __init__(self, veh_path_data, route_data, sites_data, kmz_file):
        self.animation_file = kmz_file
        self.path = open(veh_path_data)
        self.route = route_data
        self.sites = sites_data

        try:
            self.intruders_file = open("intruder_history.txt", 'r')
        except IOError as e:
            print "KmzWriter: could not open intruder history output file."
            pass

        self.feetToMeters = 0.3048

        # These values will be read from the ditch sites file
        self.lonLatAltStrings = []
        self.lonLatAltScaled = []
        self.listOfXLocations = []

        # These values are set later if a big X is being placed as a designator
        # Default lat/lon is on Langley runway, so Google Earth won't send us to south pole
        # if X is printed in error
        self.addX = False
        self.XLat = 37.077857
        self.XLon = -76.375968

        # difference between epoch time and sim time for syncing files
        epochSimDelta = 0.0

    def setAnimationName(self, nameIn):
        self.animation_file = nameIn

    def printFirstSection(self):
        self.kmz = open(self.animation_file,"w")

        # Print the non-dynamic kmz file info needed up front
        self.kmz.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n")
        self.kmz.write("<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\" xmlns:kml=\"http://www.opengis.net/kml/2.2\" xmlns:atom=\"http://www.w3.org/2005/Atom\">\n")
        self.kmz.write("  <Folder>\n")
        self.kmz.write("    <Style id=\"perimeterPolyStyle\">\n")
        self.kmz.write("      <PolyStyle>\n")
        self.kmz.write("	<fill>0</fill><outline>1</outline>\n")
        self.kmz.write("	<colorMode>normal</colorMode>\n")
        self.kmz.write("      </PolyStyle>\n")
        self.kmz.write("    </Style>\n")
        
        self.kmz.write("    <Folder><name>LaRC_Routes</name>\n")
        self.kmz.write("      <Style id=\"arrivalRouteStyle\">\n")
        self.kmz.write("	<LineStyle>\n")
        self.kmz.write("	  <color>7f00ffff</color>\n")
        self.kmz.write("	  <width>4</width>\n")
        self.kmz.write("	</LineStyle>\n")
        self.kmz.write("	<PolyStyle>\n")
        self.kmz.write("	  <color>7f00ff00</color>\n")
        self.kmz.write("	</PolyStyle>\n")
        self.kmz.write("      </Style> \n")
        self.kmz.write("      <!-- Route display -->\n")
        self.kmz.write("      <Placemark>\n")
        self.kmz.write("	<name>Route_1</name>\n")
        self.kmz.write("	<styleUrl>#arrivalRouteStyle</styleUrl>\n")
        self.kmz.write("	<MultiGeometry><LineString>\n")
        self.kmz.write("	    <altitudeMode>relativeToGround</altitudeMode>\n")
        self.kmz.write("	    <coordinates>\n")
        
    def printRoute(self):
        self.kmz.write("	      <!-- units for altitude in meters (I think)-->\n")
        for x in self.lonLatAltScaled:
            self.kmz.write("	      " + x + "\n")

    def printCloseRoutes(self):
        self.kmz.write("	   </coordinates></LineString></MultiGeometry>\n")
        self.kmz.write("       </Placemark>\n")
        self.kmz.write("    </Folder>\n")

    def printSimulatedFlightHeader(self):
        self.kmz.write("    <!-- simulated flight with time -->\n")
        self.kmz.write("    <Folder>\n")
        self.kmz.write("      <name>Flights_without_info</name>\n")
        self.kmz.write("      <Style id=\"KJFK\"><IconStyle><scale>0.7</scale><Icon><href>images/quadcopter.png</href></Icon></IconStyle><LabelStyle><color>f0ffffff</color><scale>0.6</scale></LabelStyle><bgColor>ffffffbb</bgColor><LineStyle><color>8000ff00</color><width>2</width></LineStyle><PolyStyle><color>7f00ff00</color></PolyStyle></Style>\n")
        self.kmz.write("      <Placemark>\n")
        self.kmz.write("	<styleUrl>#KJFK</styleUrl>\n")
        self.kmz.write("	<name>S2D</name>\n")
	self.kmz.write("	<Style>\n")
	self.kmz.write("	  <IconStyle><scale>1.0</scale>\n")
	self.kmz.write("	    <Icon>\n")
	self.kmz.write("	      <href>https://cdn.onlinewebfonts.com/svg/img_455169.png</href>\n")
	self.kmz.write("	    </Icon>\n")
	self.kmz.write("	  </IconStyle>\n")
	self.kmz.write("	</Style>\n")
        self.kmz.write("	<gx:Track>\n")
        self.kmz.write("	  <extrude>1</extrude>\n")
        self.kmz.write("	  <altitudeMode>relativeToGround</altitudeMode>\n")

    def printSimulatedFlightData(self):
        # This static print will be replaced with the flight path dynamic values from file
        # once that file is ready
        self.kmz.write("	  <when>2016-08-15T14:50:10Z</when><when>2016-08-15T14:50:20Z</when><when>2016-08-15T14:50:30Z</when><when>2016-08-15T14:50:40Z</when><when>2016-08-15T14:51:40Z</when><when>2016-08-15T14:52:40Z</when><when>2016-08-15T14:53:40Z</when><when>2016-08-15T14:54:40Z</when><when>2016-08-15T14:55:20Z</when><when>2016-08-15T14:55:30Z</when><when>2016-08-15T14:55:40Z</when><when>2016-08-15T14:55:50Z</when><gx:coord>-76.382485 37.095157 0.0</gx:coord><gx:coord>-76.383725 37.095251 30.48</gx:coord><gx:coord>-76.384143 37.094707 30.48</gx:coord><gx:coord>-76.385044 37.094584 30.48</gx:coord><gx:coord>-76.385609 37.094846 30.48</gx:coord><gx:coord>-76.386763 37.094331 30.48</gx:coord><gx:coord>-76.388863 37.095704 30.48</gx:coord><gx:coord>-76.386160 37.098651 30.48</gx:coord><gx:coord>-76.384698 37.098921 30.48</gx:coord><gx:coord>-76.384287 37.099422 30.48</gx:coord><gx:coord>-76.384465 37.101665 30.48</gx:coord><gx:coord>-76.387211 37.103353 0.0</gx:coord></gx:Track>\n")

    def printSimulatedFlightEnding(self):
        self.kmz.write("	\n")
        self.kmz.write("      </Placemark>\n")
        self.kmz.write("      </Folder>\n")
        self.kmz.write("      \n")

    def printDitchSitesHeader(self,siteName,radiusString,slat,slon,color='green'):
        self.kmz.write("      <!-- Ditch Sites as circles -->\n")
        self.kmz.write("      <Placemark>\n")
        self.kmz.write("	<styleUrl>#KJFK</styleUrl>\n")
        self.kmz.write("	<name>" + siteName + "</name>\n")
        self.kmz.write("	<visibility>1</visibility>\n")
        self.kmz.write("	<Style>\n")
        self.kmz.write("	  <IconStyle><scale>0.2</scale>\n")
        self.kmz.write("	    <Icon>\n")
        self.kmz.write("	      <href>http://maps.google.com/mapfiles/kml/shapes/cross-hairs_highlight.png</href>\n")
        self.kmz.write("	    </Icon>\n")
        self.kmz.write("	  </IconStyle>\n")
        self.kmz.write("	</Style>\n")
        self.kmz.write("	<Point>\n")
        self.kmz.write("	  <coordinates>" + slon + "," + slat + "</coordinates>\n")
        self.kmz.write("	</Point>\n")
        self.kmz.write("      </Placemark>\n")
        self.kmz.write("      <Placemark>\n")
        self.kmz.write("	<name>" + siteName + "</name>\n")
        self.kmz.write("	<description><![CDATA[radius " + radiusString + " feet<P>Generated by <a href='http://kml4earth.appspot.com/'>Kml4Earth</a>]]></description>\n")
        self.kmz.write("	<Style>\n")
        self.kmz.write("	  <IconStyle>\n")
        self.kmz.write("	    <Icon/>\n")
        self.kmz.write("	  </IconStyle>\n")
        self.kmz.write("	  <LineStyle>\n")
        if color == "green":
            self.kmz.write("	    <!-- Green -->\n")
            self.kmz.write("	    <color>ff00ff00</color>\n")
        elif color == "red":
            self.kmz.write("	  <!-- red -->\n")
            self.kmz.write("	  <color>ff0000ff</color>\n")
        elif color == "blue":
            self.kmz.write("	  <color>ffff0000</color>\n")

        self.kmz.write("	    <width>2</width>\n")
        self.kmz.write("	  </LineStyle>\n")
        self.kmz.write("	</Style>\n")
        self.kmz.write("	<LineString>\n")
        self.kmz.write("	  <tessellate>1</tessellate>\n")
        self.kmz.write("	  <altitudeMode>relativeToGround</altitudeMode>\n")
        self.kmz.write("	  <coordinates>\n")

    def printDitchSitesEnding(self):
        self.kmz.write("          </coordinates>\n")
        self.kmz.write("	</LineString>\n")
        self.kmz.write("      </Placemark>\n")
        self.kmz.write("   \n")


    def printDitchSites(self):
        # This method is no long used. Draws static circles for testing.
        self.kmz.write("      <!-- Ditch Sites as circles -->\n")
        self.kmz.write("      <!-- Draw green circle -->\n")
        self.kmz.write("      <Placemark>\n")
        self.kmz.write("	<name>Circle</name>\n")
        self.kmz.write("	<description><![CDATA[radius 100 feet<P>Generated by <a href='http://kml4earth.appspot.com/'>Kml4Earth</a>]]></description>\n")
        self.kmz.write("	<Style>\n")
        self.kmz.write("	  <IconStyle>\n")
        self.kmz.write("	    <Icon/>\n")
        self.kmz.write("	  </IconStyle>\n")
        self.kmz.write("	  <LineStyle>\n")
        self.kmz.write("	    <!-- Green -->\n")
        self.kmz.write("	    <color>ff00ff00</color>\n")
        self.kmz.write("	    <width>2</width>\n")
        self.kmz.write("	  </LineStyle>\n")
        self.kmz.write("	</Style>\n")
        self.kmz.write("	<LineString>\n")
        self.kmz.write("	  <tessellate>1</tessellate>\n")
        self.kmz.write("	  <coordinates>-76.38213914530199,37.095150999504376,0.0 -76.38214297452333,37.09519193330289,0.0 -76.38215437702571,37.095231952729,0.0 -76.38217309810716,37.09527016381274,0.0 -76.38219871957861,37.095305712978366,0.0 -76.38223066910487,37.095337806112035,0.0 -76.38226823298908,37.09536572630147,0.0 -76.38231057211517,37.09538884985093,0.0 -76.3823567406926,37.095406660214,0.0 -76.38240570738394,37.09541875953284,0.0 -76.38245637834412,37.095424877525886,0.0 -76.38250762165588,37.095424877525886,0.0 -76.38255829261607,37.09541875953283,0.0 -76.38260725930742,37.095406660214,0.0 -76.38265342788482,37.09538884985093,0.0 -76.38269576701093,37.09536572630147,0.0 -76.38273333089512,37.09533780611204,0.0 -76.38276528042138,37.095305712978366,0.0 -76.38279090189283,37.09527016381274,0.0 -76.38280962297428,37.095231952728994,0.0 -76.38282102547667,37.09519193330288,0.0 -76.38282485469799,37.09515099950438,0.0 -76.38282102511194,37.0951100657276,0.0 -76.38280962227722,37.09507004636477,0.0 -76.38279090092539,37.09503183538022,0.0 -76.3827652792695,37.094996286340894,0.0 -76.38273332966116,37.0949641933494,0.0 -76.38269576580453,37.09493627330539,0.0 -76.38265342681319,37.094913149891696,0.0 -76.38260725846575,37.09489533964265,0.0 -76.38255829207915,37.09488324040596,0.0 -76.38250762147145,37.0948771224559,0.0 -76.38245637852854,37.09487712245589,0.0 -76.38240570792084,37.09488324040596,0.0 -76.38235674153425,37.09489533964263,0.0 -76.38231057318681,37.09491314989169,0.0 -76.38226823419546,37.09493627330539,0.0 -76.38223067033884,37.0949641933494,0.0 -76.38219872073049,37.094996286340894,0.0 -76.3821730990746,37.09503183538022,0.0 -76.38215437772277,37.095070046364775,0.0 -76.38214297488807,37.0951100657276,0.0 -76.38213914530199,37.095150999504376,0.0</coordinates>\n")
        self.kmz.write("	</LineString>\n")
        self.kmz.write("      </Placemark>\n")
        self.kmz.write("      <Placemark>\n")
        self.kmz.write("	<name>center point</name>\n")
        self.kmz.write("	<visibility>0</visibility>\n")
        self.kmz.write("	<Style>\n")
        self.kmz.write("	  <IconStyle>\n")
        self.kmz.write("	    <Icon>\n")
        self.kmz.write("	      <href>http://maps.google.com/mapfiles/kml/shapes/cross-hairs_highlight.png</href>\n")
        self.kmz.write("	    </Icon>\n")
        self.kmz.write("	  </IconStyle>\n")
        self.kmz.write("	</Style>\n")
        self.kmz.write("	<Point>\n")
        self.kmz.write("	  <coordinates>-76.382487,37.095245</coordinates>\n")
        self.kmz.write("	</Point>\n")
        self.kmz.write("      </Placemark>\n")
        self.kmz.write("      \n")
        self.kmz.write("    </Folder>\n")
        self.kmz.write("    \n")
        self.kmz.write("    <!-- Draw red circle -->\n")
        self.kmz.write("    <Placemark>\n")
        self.kmz.write("      <name>Circle</name>\n")
        self.kmz.write("      <description><![CDATA[radius 50 feet<P>Generated by <a href='http://kml4earth.appspot.com/'>Kml4Earth</a>]]></description>\n")
        self.kmz.write("      <Style>\n")
        self.kmz.write("	<IconStyle>\n")
        self.kmz.write("          <Icon/>\n")
        self.kmz.write("	</IconStyle>\n")
        self.kmz.write("	<LineStyle>\n")
        self.kmz.write("	  <!-- red -->\n")
        self.kmz.write("	  <color>ff0000ff</color>\n")
        self.kmz.write("	  <width>2</width>\n")
        self.kmz.write("	</LineStyle>\n")
        self.kmz.write("      </Style>\n")
        self.kmz.write("      <LineString>\n")
        self.kmz.write("	<tessellate>1</tessellate>\n")
        self.kmz.write("	<coordinates> -76.38359657251358,37.09521199987609,0.0 -76.38359848717138,37.095232466772416,0.0 -76.38360418846868,37.09525247647735,0.0 -76.3836135490507,37.09527158200663,0.0 -76.38362635981976,37.09528935657347,0.0 -76.38364233460597,37.095305403122374,0.0 -76.38366111655967,37.09531936319876,0.0 -76.38368228612285,37.0953309249564,0.0 -76.3837053704013,37.0953398301236,0.0 -76.38372985372851,37.09534587977268,0.0 -76.38375518918485,37.09534893876381,0.0 -76.38378081081517,37.09534893876381,0.0 -76.38380614627151,37.09534587977268,0.0 -76.38383062959872,37.095339830123606,0.0 -76.38385371387717,37.0953309249564,0.0 -76.38387488344036,37.09531936319876,0.0 -76.38389366539406,37.095305403122374,0.0 -76.38390964018026,37.09528935657347,0.0 -76.3839224509493,37.09527158200663,0.0 -76.38393181153133,37.09525247647735,0.0 -76.38393751282864,37.095232466772416,0.0 -76.38393942748642,37.09521199987609,0.0 -76.38393751273745,37.0951915329852,0.0 -76.38393181135706,37.09517152329608,0.0 -76.38392245070744,37.0951524177916,0.0 -76.38390963989228,37.09513464325634,0.0 -76.38389366508557,37.09511859674298,0.0 -76.38387488313874,37.095104636702935,0.0 -76.38385371360926,37.095093074979246,0.0 -76.3838306293883,37.09508416984055,0.0 -76.38380614613729,37.095078120212,0.0 -76.38378081076905,37.09507506123163,0.0 -76.38375518923097,37.095075061231626,0.0 -76.38372985386273,37.095078120212,0.0 -76.38370537061172,37.09508416984055,0.0 -76.38368228639075,37.095093074979246,0.0 -76.38366111686128,37.09510463670295,0.0 -76.38364233491446,37.09511859674298,0.0 -76.38362636010773,37.09513464325634,0.0 -76.38361354929258,37.0951524177916,0.0 -76.38360418864296,37.09517152329608,0.0 -76.38359848726257,37.0951915329852,0.0 -76.38359657251358,37.09521199987609,0.0</coordinates>\n")
        self.kmz.write("      </LineString>\n")
        self.kmz.write("    </Placemark>\n")
        self.kmz.write("    <Placemark>\n")
        self.kmz.write("      <name>center point</name>\n")
        self.kmz.write("      <visibility>0</visibility>\n")
        self.kmz.write("      <Style>\n")
        self.kmz.write("	<IconStyle>\n")
        self.kmz.write("	  <Icon>\n")
        self.kmz.write("	    <href>http://maps.google.com/mapfiles/kml/shapes/cross-hairs_highlight.png</href>\n")
        self.kmz.write("	  </Icon>\n")
        self.kmz.write("	</IconStyle>\n")
        self.kmz.write("      </Style>\n")
        self.kmz.write("      <Point>\n")
        self.kmz.write("	<coordinates>-76.383768,37.095212</coordinates>\n")
        self.kmz.write("      </Point>\n")
        self.kmz.write("    </Placemark>\n")
        self.kmz.write("\n")
        self.kmz.write("    <!-- Blue Circle -->\n")
        self.kmz.write("    <Placemark>\n")
        self.kmz.write("      <name>Circle</name>\n")
        self.kmz.write("      <description><![CDATA[radius 50 feet<P>Generated by <a href='http://kml4earth.appspot.com/'>Kml4Earth</a>]]></description>\n")
        self.kmz.write("      <Style>\n")
        self.kmz.write("	<IconStyle>\n")
        self.kmz.write("          <Icon/>\n")
        self.kmz.write("	</IconStyle>\n")
        self.kmz.write("	<LineStyle>\n")
        self.kmz.write("	  <color>ffff0000</color>\n")
        self.kmz.write("	  <width>2</width>\n")
        self.kmz.write("	</LineStyle>\n")
        self.kmz.write("      </Style>\n")
        self.kmz.write("      <LineString>\n")
        self.kmz.write("	<tessellate>1</tessellate>\n")
        self.kmz.write("	<coordinates> -76.38664213337964,37.09779699950433,0.0 -76.38664596273411,37.09783793328454,0.0 -76.386657365633,37.09787795269274,0.0 -76.38667608736543,37.09791616375941,0.0 -76.38670170972782,37.09795171290913,0.0 -76.3867336603651,37.097983806028466,0.0 -76.38677122555552,37.098011726205414,0.0 -76.38681356615392,37.09803484974453,0.0 -76.3868597363368,37.09805266009965,0.0 -76.38690870473093,37.09806475941306,0.0 -76.38695937745312,37.09807087740339,0.0 -76.38701062254684,37.09807087740339,0.0 -76.38706129526906,37.09806475941306,0.0 -76.38711026366317,37.09805266009965,0.0 -76.38715643384606,37.098034849744536,0.0 -76.38719877444446,37.098011726205414,0.0 -76.3872363396349,37.097983806028466,0.0 -76.38726829027217,37.09795171290913,0.0 -76.38729391263458,37.09791616375941,0.0 -76.38731263436698,37.09787795269274,0.0 -76.38732403726587,37.09783793328454,0.0 -76.38732786662035,37.09779699950433,0.0 -76.38732403690108,37.09775606574587,0.0 -76.38731263366984,37.09771604640095,0.0 -76.38729391166699,37.09767783543348,0.0 -76.38726828912013,37.09764228641007,0.0 -76.38723633840077,37.09761019343293,0.0 -76.38719877323791,37.09758227340142,0.0 -76.38715643277429,37.09755914999808,0.0 -76.38711026282141,37.09754133975699,0.0 -76.38706129473209,37.09752924052573,0.0 -76.3870106223624,37.097523122578394,0.0 -76.3869593776376,37.0975231225784,0.0 -76.3869087052679,37.09752924052573,0.0 -76.38685973717858,37.09754133975699,0.0 -76.3868135672257,37.09755914999807,0.0 -76.38677122676208,37.09758227340142,0.0 -76.3867336615992,37.09761019343293,0.0 -76.38670171087985,37.09764228641007,0.0 -76.386676088333,37.09767783543348,0.0 -76.38665736633014,37.09771604640095,0.0 -76.3866459630989,37.09775606574587,0.0 -76.38664213337964,37.09779699950433,0.0</coordinates>\n")
        self.kmz.write("      </LineString>\n")
        self.kmz.write("    </Placemark>\n")
        self.kmz.write("    <Placemark>\n")
        self.kmz.write("      <name>center point</name>\n")
        self.kmz.write("      <visibility>0</visibility>\n")
        self.kmz.write("      <Style>\n")
        self.kmz.write("	<IconStyle>\n")
        self.kmz.write("	  <Icon>\n")
        self.kmz.write("	    <href>http://maps.google.com/mapfiles/kml/shapes/cross-hairs_highlight.png</href>\n")
        self.kmz.write("	  </Icon>\n")
        self.kmz.write("	</IconStyle>\n")
        self.kmz.write("      </Style>\n")
        self.kmz.write("      <Point>\n")
        self.kmz.write("	<coordinates>-76.386929,37.097777</coordinates>\n")
        self.kmz.write("      </Point>\n")
        self.kmz.write("    </Placemark>\n")

    def printCloseKmlFile(self):
        self.kmz.write("  </Folder>\n")
        self.kmz.write("</kml>\n")

#Global function for units conversion
    def getRadius(self,unit): 
        r = {'KM': 6371.009, 'MI': 3958.761, 'NM': 3440.070, 'YD': 6967420, 'FT': 20902260} 
        if unit in r: 
            return r[unit] 
        else: 
            return unit 
    
    # Function to compute new lat/lon from centroid, bearing, and radius
    # Called multiple times for each ditch site to define the circumference of a circle
    def destination(self,lat1, lon1, alt, brng, dt, unit='KM'): 
        r = self.getRadius(unit) 
        lat1 = math.radians(lat1) 
        lon1 = math.radians(lon1) 
        lat3 = math.asin(math.sin(lat1) * math.cos(dt / r) + math.cos(lat1) * math.sin(dt / r) * math.cos(math.radians(brng))) 
        lon3 = lon1 + math.atan2(math.sin(math.radians(brng)) * math.sin(dt / r) * math.cos(lat1) , math.cos(dt / r) - math.sin(lat1) * math.sin(lat3)) 
        
        return math.degrees(lat3),math.degrees(lon3), alt

    def createAnimationWithXs(self, listOfBigXs):
        self.addX = True
        self.listOfXLocations = listOfBigXs
        self.createAnimation()

    def createAnimation(self):
        # Write out static first section of kmz file, includes header for route output
        self.printFirstSection()

        # Scale altitude from route input file from feet to meters, and then
        # build the string to write to the kmz file
        for this_entry in range(len(self.route)):
            alt_meters = self.route[this_entry].alt * self.feetToMeters
            newString = str(math.degrees(self.route[this_entry].lon)) + "," + str(math.degrees(self.route[this_entry].lat)) + "," + str(alt_meters)
            self.lonLatAltScaled.append(newString)

        #self.lonLatAltStrings = self.route_file.readlines()
        #for lla_line in self.lonLatAltStrings:
        #    latstring, lonstring, altstring = lla_line.split(",")
        #    alt_meters = float(altstring) * self.feetToMeters
        #    strAltMeters = str(alt_meters)
        #    newString = latstring + "," + lonstring + "," + strAltMeters
        #    self.lonLatAltScaled.append(newString)

        # Write route to kmz file
        self.printRoute()
        self.printCloseRoutes()

        # Write flight track to kmz file
        self.printSimulatedFlightHeader()

        # Create the dynamic part of the KMZ output for the track
        tracks = self.path.readlines()
        whenTokens = []     # list to hold time token strings
        coordTokens = []    # list to hold coordinate token strings

        hours_prev = -1
        minutes_prev = -1
        seconds_prev = -1

        timeOffsetComputed = False

        for track in tracks:
            timeepoch, timestr, flatstr, flonstr, faltstr = track.split()
            ftimeepoch = float(timeepoch)
            timeInSec = float(timestr)
            flat = float(flatstr)
            flon = float(flonstr)
            falt = float(faltstr) * self.feetToMeters
            
            # for first line only, compute offset of epoch time versus sim time,
            # which is needed to sync some components
            if not timeOffsetComputed:
                self.epochSimDelta = ftimeepoch - timeInSec
                timeOffsetComputed = True
    
            # Compute hours, minutes, seconds from timeint, needed to build
            # kml string, which is xx:xx:xx format
            hours = int(timeInSec/3600)
            minutes = int((timeInSec - hours * 3600)/60)
            seconds = int((timeInSec - hours * 3600 - minutes * 60))

            # KMZ can only handle as fine grained as seconds, so if there is a 
            # previously read line with same hr:min:sec, toss this one away. This 
            # will reduce the KMZ print to only include the first time entry for any 
            # given hr:min:sec
            if hours == hours_prev and minutes == minutes_prev and seconds == seconds_prev:
                use_line = False
            else:
                use_line = True

            if use_line:
                # Start of kmz token
                kmzTimeStr = "<when>2016-08-15T"
                # Add hours, minutes, seconds with filling zeros where needed
                if hours < 10:
                    kmzTimeStr = kmzTimeStr + "0" + str(hours) + ":"
                else:
                    kmzTimeStr = kmzTimeStr + str(hours) + ":"
                    
                if minutes < 10:
                    kmzTimeStr = kmzTimeStr + "0" + str(minutes) + ":"
                else:
                    kmzTimeStr = kmzTimeStr + str(minutes) + ":"
            
                # Pack closing when-token after seconds
                if seconds < 10:
                    kmzTimeStr = kmzTimeStr + "0" + str(seconds) + "Z</when>"
                else:
                    kmzTimeStr = kmzTimeStr + str(seconds) + "Z</when>"
    
                whenTokens.append(kmzTimeStr)

                # Build gx:coord token for kmz
                # <gx:coord>-76.382485 37.095157 0.0</gx:coord>
                coordString = "<gx:coord>"+str(flon)+" "+str(flat)+" "+str(falt)+"</gx:coord>"

                coordTokens.append(coordString)

            hours_prev   = hours
            minutes_prev = minutes
            seconds_prev = seconds

        # Now dump out the list elements in the correct order for kml
        for index in range(len(whenTokens)):
            self.kmz.write(whenTokens[index])

        for index in range(len(coordTokens)):
            self.kmz.write(coordTokens[index])

        self.kmz.write("</gx:Track>\n")

        # Finish the track section of the kmz file by adding the static closing data
        self.printSimulatedFlightEnding()

        centerLat = 37.098921
        centerLon = -76.384698
        bearingIncrement = 10

        #lines = self.sites.readlines()
        #for line in lines:
        #    namestr,typestr,lonstr,latstr,altstr,radiusstr,percentstr = line.split(",")
        for line in range(len(self.sites)):
            namestr = self.sites[line].name
            typeint = self.sites[line].type
            centerLat = math.degrees(self.sites[line].lat)
            centerLon = math.degrees(self.sites[line].lon)
            centerAlt = self.sites[line].alt * self.feetToMeters
            radiusFt = self.sites[line].radius
            percentIn = self.sites[line].reliability

            # Assigning color based on very large granularity of goodness percentage for now
            if percentIn > 75.0 : colorPrint = "green"
            elif percentIn > 40.0 : colorPrint = "blue"
            else : colorPrint = "red"

            # Write the header for each circle display to kmz file
            self.printDitchSitesHeader(namestr,str(radiusFt),str(centerLat),str(centerLon),colorPrint)
            out = [0.0, 0.0, 0.0]
            thisBearing = 0
            # Find lat/lon at discrete intervals to define the outside of the circle
            # that displays the ditch site on Google Earth
            while (thisBearing < 361.0):
                out = self.destination(centerLat, centerLon, centerAlt, thisBearing, radiusFt, "FT")
                # Write lat/lon/alt values to kmz file
                self.kmz.write("             {d[1]},{d[0]},{d[2]}\n".format(d=out))
                thisBearing = thisBearing + bearingIncrement

            # Write closing lines for the ditch sites folder to the kmz output
            self.printDitchSitesEnding()

        # If a big X lat/lon was set, add that to the animation
        if self.addX:
            for thisX in range(len(self.listOfXLocations)):
                self.kmz.write("      <Placemark>\n")
                self.kmz.write("	<Style>\n")
                self.kmz.write("	  <scale>2.0</scale>\n")
                self.kmz.write("	</Style>\n")
                self.kmz.write("	<Point>\n")
                self.kmz.write("	  <coordinates>" + str(math.degrees(self.listOfXLocations[thisX].lon)) + "," + str(math.degrees(self.listOfXLocations[thisX].lat)) + "</coordinates>\n")
                self.kmz.write("	</Point>\n")
                self.kmz.write("      </Placemark>\n")
                
        print "kmz_writer: before process_intruders"
        self.process_intruders()
            
        # Write the closing tokens to the kmz file
        self.printCloseKmlFile()
        
    def process_intruders(self):
        intruder_lines = self.intruders_file.readlines()

        # list to hold intruder objects
        intruders = []

        # input line in intruder history file will look like this (2 intruders here)
        # (epoch time   int1_lat   int1_lon  int1_alt int2_lat   int2_lon  int2_alt  ...)
        # 1517603289.05 37.102051 -76.387308 0.000000 37.102053 -76.387314 0.000000 
        for this_entry in intruder_lines:
            pieces = this_entry.split()
            timeepoch = pieces[0]
            ftimeepoch = float(timeepoch)
            timeInSec = ftimeepoch - self.epochSimDelta
            if timeInSec < 0.0:
                timeInSec = 0.0

            # pack seconds into a kmz string of hh:mm:sec
            hours = int(timeInSec/3600)
            minutes = int((timeInSec - hours * 3600)/60)
            seconds = int((timeInSec - hours * 3600 - minutes * 60))

            kmzTimeStr = ""
            # Add hours, minutes, seconds with filling zeros where needed
            if hours < 10:
                kmzTimeStr = kmzTimeStr + "0" + str(hours) + ":"
            else:
                kmzTimeStr = kmzTimeStr + str(hours) + ":"
                
            if minutes < 10:
                kmzTimeStr = kmzTimeStr + "0" + str(minutes) + ":"
            else:
                kmzTimeStr = kmzTimeStr + str(minutes) + ":"

            if seconds < 10:
                kmzTimeStr = kmzTimeStr + "0" + str(seconds)
            else:
                kmzTimeStr = kmzTimeStr + str(seconds)
    
            # remove the time entry, so everything left will be related to lat/lon/alt
            del pieces[0]

            # this "pieces" list will have sequentially the lat, lon, alt as elements
            num_intruders = (len(pieces))/3
            num_to_add = num_intruders - len(intruders)

            if (num_to_add > 0):
                for i in range(num_to_add):
                    new_name = str(num_intruders + i)
                    new_intruder = Intruder(str(new_name))
                    intruders.append(new_intruder)

            # now assemble sequential set of 3 pieces into each gxcoord set
            # swap elements for kmz, also (lon, lat, alt from lat, lon, alt)
            for i in range(0, num_intruders):
                new_coord_set = pieces[i*3+1] + " " + pieces[i*3+0] + " " + pieces[i*3+2]
                intruders[i].add_when(str(kmzTimeStr))
                intruders[i].add_gxcoord(new_coord_set)

        # print out each intruder to kmz
        for i in range(len(intruders)):
            self.print_intruder(intruders[i])

    def print_intruder(self, int_in):
        opening_str = '      <Folder>\n        <name>Intruders</name>\n        <Style id="KJFK"><IconStyle><scale>0.7</scale><Icon><href>http://maps.google.com/mapfiles/kml/paddle/red-stars.png</href></Icon></IconStyle><LabelStyle><color>f0ffffff</color><scale>0.8</scale></LabelStyle><bgColor>ffffffbb</bgColor><LineStyle><color>8000ff00</color><width>2</width></LineStyle><PolyStyle><color>7f00ff00</color></PolyStyle></Style>\n        <Placemark>\n      	<styleUrl>#KJFK</styleUrl>\n      	<name>Int</name>\n      	<gx:Track>\n      	<extrude>1</extrude>\n      	<altitudeMode>relativeToGround</altitudeMode>\n'

        closing_str = '</gx:Track>\n          </Placemark>\n       </Folder>\n'

        all_whens = ""
        all_gxcoords = ""
        # Build a single long string for each the time values and coords, as kmz wants it
        for i in range(len(int_in.get_when())):
            # append newest time and position to each kmz string
            all_whens = all_whens + '<when>2016-08-15T'+ int_in.get_when()[i] + 'Z</when>'
            all_gxcoords = all_gxcoords + '<gx:coord>' + int_in.get_gxcoord()[i] + '</gx:coord>'

        self.kmz.write(opening_str + all_whens + '\n' + all_gxcoords + '\n' + closing_str)

    def run_no_engage(self):
        self.createAnimation()

    def run(self, listOfXs):
        # Adds a big "X" to the site denoted in the input args
        self.createAnimationWithXs(listOfXs)


if __name__ == '__main__':

    writer = KmzWriter('RouteFile2.txt', 'path_output_v2.txt', 'LangleySitesV1.txt', animation_file_name)
    writer.createAnimation()
