# SpexiNetworkExplorerTools

Helper to visualize and plan Spexi missions


Features:

- 3D Terrain Toggle
  Toggle 3D terrain on the map using the new 3D button at the bottom right

- Mission List
  When selecting a Spexigon, list all available missions with details about the type, reward, and creation time.

- Flight Plan Export and Preview
  Export an available mission's flight plan using the new Export button. Current format support: .kml (Google Earth), and .geojson. All mission types supported: Map, Multi Panorama, Panorama, and Grid Map. Multi Panorama and Panorama are accurate to <2m, although the order of the Multi Panorama waypoints will most likely be wrong. Map and Grid Map export should only be used for planning (scouting out pilot position), the actual flight paths are likely to be wrong in some way.
  Preview an available mission's flight plan on the Spexi Network Explorer using the new Preview button. Panorama and Multi Panorama missions only render the pano locations at the correct location and altitude. Map and Grid Map missions only render the flight paths at ground level, if you want 3D visualization, use Export.


Installation:

Step 1: Install Tampermonkey Extension
Chrome or Chromium based browsers: https://chromewebstore.google.com/detail/dhdgffkkebhmkfjojejmpbldmpobfkfo
Firefox: https://addons.mozilla.org/en-CA/firefox/addon/tampermonkey/
Edge: https://microsoftedge.microsoft.com/addons/detail/tampermonkey/iikmkjmpaadaobahmlepeloendndfphd

Step 2: Install The Script
on [this](https://greasyfork.org/en/scripts/533567-spexi-network-explorer-tools) page, click Install this script, and follow additional instructions.
