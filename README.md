# SpexiNetworkExplorerTools

Helper to visualize and plan Spexi missions


Features:

- List available missions for a Spexigon, including the mission type, reward, and conversion rate to local currency.
- Countdown timer for reservation expiration.
- Add "Failed" and "Taken" flights to Flight History section for a Spexigon.
- Mission geometry export: export missions to Google Earth for planning purposes.

Mission export:

Mission export derives directly from the Spexi App to produce 1:1 mission geometry*. With a Spexigon selected, click "Download Flight Path" next to where the Reserve button is. Choose any of the mission types to download instantly. Since mission geometry is based off drone sensor size, different drones will produce different geometries. To change your selected drone, navigate "Download Flight Path" > "⚙️ Settings" and select your drone. To change Trusted Pilot Parameters, navigate to ⚙️ Settings and move the sliders to your desired parameters. The Min/Max/Step Size of these parameters should match the Spexi App. Note there is no Front Overlap parameter since it does not affect flight geometry; however it does affect photo count which is todo.

To perform a quick download of an available mission, click the 📥 KML button next to an available mission. This generated mission uses last saved flight parameters.

*Small differences in the Math engines between this script and the Spexi App may produce inaccuracies of up to 1 meter, and for Grid Map missions photo counts +-2.

Installation:

Step 1: Install Tampermonkey Extension
Chrome or Chromium based browsers: https://chromewebstore.google.com/detail/dhdgffkkebhmkfjojejmpbldmpobfkfo
Firefox: https://addons.mozilla.org/en-CA/firefox/addon/tampermonkey/
Edge: https://microsoftedge.microsoft.com/addons/detail/tampermonkey/iikmkjmpaadaobahmlepeloendndfphd

Step 2: Install The Script
on [this](https://greasyfork.org/en/scripts/533567-spexi-network-explorer-tools) page, click Install this script, and follow additional instructions.
