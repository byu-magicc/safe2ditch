Rock Canyon Park Gazebo Model
=============================

Created from Google Maps, zoom level = 20. I used [this](http://www.wolfpil.de/v3/deep-zoom.html) guy's website, which is pretty much also accessible via the [official Google API docs](https://developers.google.com/maps/documentation/javascript/examples/maxzoom-simple). Before making the map full screen, I loaded the Chrome Developer Tools and undocked so that I could access the HTML DOM while the map was in full screen (a machine with two monitors). Under `Satellite`, uncheck `Labels` to hide street names. Click the full screen button on the map. Begin your search in the `Elements` tab of the Developer Tools to delete the `<div>` elements. As you hover over the elements you should watch them become highlighted on the map. Once you have deleted all the text on top of the map, frame the map and take a screen shot. Make sure that you have at least one landmark that you can use to register all the images together with.

I then used GIMP to stitch them together, using the landmarks I picked in the image. Using Google Map's "Measure distance" option when you right click (on https://maps.google.com), I found the dimensions of my image in meters. I then plugged these into the model information in the `model.sdf`.

When you include this model in a world using

```xml
<include>
  <uri>model://rock_canyon_park</uri>
  <!-- This pose is meant to line up with 40.267987,-111.635558 in copter_sitl.sh -->
  <pose>-58 -13 0 0 0 0</pose>
</include>
```

make sure to shift the pose so that the origin is lined up with the GPS coordinates you pass into `copter_sitl.sh`.