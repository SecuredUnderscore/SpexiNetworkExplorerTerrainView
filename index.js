// ==UserScript==
// @name         Spexi Network Explorer Tools
// @author       Secured_ on Discord
// @namespace    http://tampermonkey.net/
// @version      3.0
// @match        https://explorer.spexi.com/*
// @require      https://unpkg.com/h3-js@4.1.0/dist/h3-js.umd.js
// @require      https://cdn.jsdelivr.net/npm/@turf/turf@6/turf.min.js
// @license      MIT
// @grant        none
// @run-at       document-start
// @description  Generate KML flight paths for Spexi Missions directly from the Explorer
// @downloadURL https://update.greasyfork.org/scripts/533567/Spexi%20Network%20Explorer%20Tools.user.js
// @updateURL https://update.greasyfork.org/scripts/533567/Spexi%20Network%20Explorer%20Tools.meta.js
// ==/UserScript==

(function () {
    'use strict';

    // ─────────────────────────────────────────────────────────────────────────────
    // User Settings & Dynamic Loading
    // ─────────────────────────────────────────────────────────────────────────────
    let DRONE_MODEL = "DJI Mini 3";

    let MAP_PARAMS = {
        altitude: 80,
        flap: 0.80,
        slap: 0.70,
        bearing: { mode: "relative", value: 0 },
        gimbalPitch: -90,
        gimbalPitchGrid: -60,
    };

    let PANO_PARAMS = {
        altitude: 80,
        speed: 10,
        fov: 360,
        overlap: 0.45,
        gimbalPitches: [-60, -30]
    };

    function loadSettings() {
        try {
            let saved = localStorage.getItem("spexi_flight_settings");
            if (saved) {
                let s = JSON.parse(saved);
                if (s.droneModel) DRONE_MODEL = s.droneModel;
                if (s.altitude !== undefined) MAP_PARAMS.altitude = PANO_PARAMS.altitude = s.altitude;
                if (s.gimbalPitch !== undefined) MAP_PARAMS.gimbalPitch = s.gimbalPitch;
                if (s.gimbalPitchGrid !== undefined) MAP_PARAMS.gimbalPitchGrid = s.gimbalPitchGrid;
                if (s.slap !== undefined) MAP_PARAMS.slap = s.slap;
            }
        } catch (e) { }
    }

    function saveSettings(s) {
        localStorage.setItem("spexi_flight_settings", JSON.stringify(s));
        loadSettings();
        let droneDisplay = document.getElementById("spexi-drone-info-text");
        if (droneDisplay) droneDisplay.innerText = `Drone: ${DRONE_MODEL}`;
    }

    loadSettings();

    const DRONES = {
        "DJI Mini 2": { model: "DJI Mini 2", cameraModel: "FC7303", sensorWidthInMeters: 0.0063, sensorHeightInMeters: 0.0047, pixelPitchInMeters: 1.6e-6, focalLengthInMeters: 0.0045, hFovInDegrees: 83.0 },
        "DJI Mini 3": { model: "DJI Mini 3", cameraModel: "FC3682", sensorWidthInMeters: 0.01, sensorHeightInMeters: 0.0075, pixelPitchInMeters: 2.4e-6, focalLengthInMeters: 0.0067, hFovInDegrees: 82.1 },
        "DJI Mini 3 Pro": { model: "DJI Mini 3 Pro", cameraModel: "FC3582", sensorWidthInMeters: 0.01, sensorHeightInMeters: 0.0075, pixelPitchInMeters: 2.4e-6, focalLengthInMeters: 0.0067, hFovInDegrees: 82.1 },
        "DJI Mini 4 Pro": { model: "DJI Mini 4 Pro", cameraModel: "FC8482", sensorWidthInMeters: 0.01, sensorHeightInMeters: 0.0075, pixelPitchInMeters: 2.4e-6, focalLengthInMeters: 0.0067, hFovInDegrees: 82.1 }
    };

    const FLIGHT_PLAN_NAMES = { 1: "Map", 2: "Multi-Panorama", 3: "Panorama", 4: "Grid Map", 5: "Hybrid" };

    // ─────────────────────────────────────────────────────────────────────────────
    // Math & Geometry
    // ─────────────────────────────────────────────────────────────────────────────
    function roundCoord(x) { return Math.round(x * 10000000) / 10000000; }
    function roundAngle(x) { return Math.round(x * 1000) / 1000; }

    function calcGsd(altitude, camera) { return (altitude / camera.focalLengthInMeters) * camera.pixelPitchInMeters; }
    function calcGroundWidth(camera, gsd) { return (camera.sensorWidthInMeters / camera.pixelPitchInMeters) * gsd; }
    function calcGroundHeight(camera, gsd) { return (camera.sensorHeightInMeters / camera.pixelPitchInMeters) * gsd; }
    function calcLineSpacing(slap, camera, gsd) { return calcGroundWidth(camera, gsd) * (1 - slap); }
    function calcPhotoInterval(camera, gsd, flap) { return calcGroundHeight(camera, gsd) * (1 - flap); }

    function bearingBetween(pt1, pt2) { return (turf.bearing(pt1, pt2) + 360) % 360; }
    function rhumbBearing(pt1, pt2) { return (turf.rhumbBearing(pt1, pt2) + 360) % 360; }
    function distanceMeters(pt1, pt2) { return turf.distance(pt1, pt2, { units: 'meters' }); }

    function pointAlongLine(p1, p2, fraction) {
        return [p1[0] + (p2[0] - p1[0]) * fraction, p1[1] + (p2[1] - p1[1]) * fraction];
    }

    function rotateCoords(coords, angleDeg, pivot) {
        if (angleDeg === 0) return coords.map(c => [...c]);
        // Note: turf.transformRotate uses positive angle = clockwise. Default pivot is centroid.
        // The script uses Rhumb rotation implicitly in Turf's transformRotate.
        const line = turf.lineString(coords);
        const rotated = turf.transformRotate(line, angleDeg, { pivot: pivot });
        return rotated.geometry.coordinates;
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // H3 / Generation Logic
    // ─────────────────────────────────────────────────────────────────────────────
    function getHexBoundary(hash) {
        let b = h3.cellToBoundary(hash);
        let coords = b.map(c => [c[1], c[0]]);
        coords.push([...coords[0]]); // close ring
        return coords;
    }

    function getHexCentroid(hash) {
        let center = h3.cellToLatLng(hash);
        return [center[1], center[0]];
    }

    function getLongestLineIndex(hexCoords) {
        let maxDist = -1, maxIdx = 0;
        for (let i = 0; i < hexCoords.length - 1; i++) {
            let d = distanceMeters(hexCoords[i], hexCoords[i + 1]);
            if (d > maxDist) { maxDist = d; maxIdx = i; }
        }
        return maxIdx;
    }

    function getHexPanos(hash) {
        const center = getHexCentroid(hash);
        const hexPoly = turf.polygon([getHexBoundary(hash)]);
        const neighbors = h3.gridDisk(hash, 1);

        let neighborCenters = [];
        for (let n of neighbors) {
            let ncList = h3.cellToLatLng(n);
            let nc = [ncList[1], ncList[0]];
            if (Math.abs(nc[0] - center[0]) < 1e-10 && Math.abs(nc[1] - center[1]) < 1e-10) continue;
            let angle = Math.atan2(nc[1] - center[1], nc[0] - center[0]);
            neighborCenters.push({ x: nc[0], y: nc[1], angle: angle });
        }

        // Match app logic: angles [0, 2PI)
        for (let n of neighborCenters) { if (n.angle < 0) n.angle += 2 * Math.PI; }
        neighborCenters.sort((a, b) => a.angle - b.angle);

        const sorted = neighborCenters.map(n => [n.x, n.y]);
        let panoPoints = [];
        for (let i = 0; i < sorted.length; i++) {
            let next_i = (i + 1) % sorted.length;
            let triangleCoords = [center, sorted[i], sorted[next_i], center];
            let triangle = turf.polygon([triangleCoords]);
            try {
                // To safely intersect poly vs poly
                let intersection = turf.intersect(hexPoly, triangle);
                if (intersection && turf.area(intersection) > 0) {
                    let c = turf.centroid(intersection);
                    panoPoints.push(c.geometry.coordinates);
                }
            } catch (e) { console.error("Intersection failed", e); }
        }
        // Rotate by -1
        if (panoPoints.length > 1) {
            panoPoints = [panoPoints[panoPoints.length - 1], ...panoPoints.slice(0, panoPoints.length - 1)];
        }
        return [center, ...panoPoints];
    }

    function getPanoActions(camera, params) {
        const fov = params.fov || 360;
        const overlap = params.overlap || 0.45;
        const hFov = camera.hFovInDegrees;
        const yawCount = Math.ceil(fov / ((1 - overlap) * hFov));
        let actions = [{ type: "SINGLE_PHOTO", yawAngle: -180, gimbalPitchAngle: -90 }];
        for (let i = 0; i < yawCount; i++) {
            let yawAngle = roundAngle(-180 + (360 * i / yawCount));
            let pitches = (i % 2 !== 0) ? [...params.gimbalPitches].reverse() : [...params.gimbalPitches];
            for (let pitch of pitches) {
                actions.push({ type: "SINGLE_PHOTO", yawAngle: yawAngle, gimbalPitchAngle: pitch });
            }
        }
        return actions;
    }

    function buildPanoWaypoint(lng, lat, altitude, speed, actions) {
        return { longitude: roundCoord(lng), latitude: roundCoord(lat), altitude, actions, speed, isFlyThrough: false, isFlightLineStart: false, isFlightLineEnd: false, imageTag: "pano" };
    }

    function generatePanoramaMission(hash, params, camera) {
        const center = getHexCentroid(hash);
        const actions = getPanoActions(camera, params);
        const wp = buildPanoWaypoint(center[0], center[1], params.altitude, params.speed, actions);
        return { type: "panorama", waypoints: [wp], pano_points: [center], flight_coords: [], photo_count: actions.length };
    }

    function generateMultiPanoramaMission(hash, params, camera) {
        const panoPoints = getHexPanos(hash);
        const actions = getPanoActions(camera, params);
        const waypoints = panoPoints.map(pt => buildPanoWaypoint(pt[0], pt[1], params.altitude, params.speed, actions));
        return { type: "multiPanorama", waypoints: waypoints, pano_points: panoPoints, flight_coords: waypoints.map(wp => [wp.longitude, wp.latitude]), photo_count: actions.length * panoPoints.length };
    }

    function generateMapPath(hash, params, camera) {
        const hexCoords = getHexBoundary(hash);
        const hexPoly = turf.polygon([hexCoords]);
        const centroid = getHexCentroid(hash);

        const longestIdx = getLongestLineIndex(hexCoords);
        let baseBearing = bearingBetween(hexCoords[longestIdx], hexCoords[longestIdx + 1]) + 90;
        let flightBearing = params.bearing.mode === "relative" ? baseBearing + params.bearing.value : params.bearing.value;
        flightBearing = (flightBearing + 360) % 360;

        const gimbalPitch = params.gimbalPitch !== undefined ? params.gimbalPitch : -90;
        const gimbalAngleFromNadir = 90 - Math.abs(gimbalPitch);
        const gimbalOffset = params.altitude * Math.tan(gimbalAngleFromNadir * Math.PI / 180);

        const gsd = calcGsd(params.altitude, camera);
        const lineSpacing = calcLineSpacing(params.slap, camera, gsd);
        const photoInterval = calcPhotoInterval(camera, gsd, params.flap);

        const isGrid = params.isGridMap || false;
        const angles = isGrid ? [0, 90] : [0];
        const allFlightCoords = [];

        for (let angle of angles) {
            let effectiveBearing = flightBearing + angle;
            // Negative effectiveBearing to align with scan direction horizontally
            let rotatedHexCoords = rotateCoords(hexCoords, -effectiveBearing, centroid);
            let rotatedPoly = turf.polygon([rotatedHexCoords]);
            let bbox = turf.bbox(rotatedPoly); // minX, minY, maxX, maxY
            let [min_x, min_y, max_x, max_y] = bbox;
            let totalHeight = max_y - min_y;

            let spacingDeg = lineSpacing / 111320;
            let numLines = Math.floor(totalHeight / spacingDeg);
            let remainder = totalHeight - (numLines * spacingDeg);
            let startOffset = spacingDeg < totalHeight ? remainder / 2 : totalHeight / 2;

            let scanLines = [];
            let lastEndpoint = null;
            let y = min_y + startOffset;

            while (y <= max_y) {
                let scanLine = turf.lineString([[min_x - 0.001, y], [max_x + 0.001, y]]);
                // Cut line by polygon
                let segments = [];
                try {
                    let split = turf.lineSplit(scanLine, rotatedPoly);
                    if (!split || split.features.length === 0) {
                        let mid = turf.midpoint(scanLine.geometry.coordinates[0], scanLine.geometry.coordinates[1]);
                        if (turf.booleanPointInPolygon(mid, rotatedPoly)) segments.push(scanLine.geometry.coordinates);
                    } else {
                        for (let feat of split.features) {
                            let pts = feat.geometry.coordinates;
                            let mid = turf.midpoint(pts[0], pts[pts.length - 1]);
                            if (turf.booleanPointInPolygon(mid, rotatedPoly)) segments.push(pts);
                        }
                    }
                } catch (e) { }

                for (let segCoords of segments) {
                    if (segCoords.length >= 2) {
                        // Reverse initially (like App DESC sort)
                        segCoords.reverse();
                        if (gimbalPitch === -90) {
                            if (scanLines.length % 2 !== 0) segCoords.reverse();
                        } else {
                            if (lastEndpoint) {
                                let dFwd = Math.pow(segCoords[0][0] - lastEndpoint[0], 2) + Math.pow(segCoords[0][1] - lastEndpoint[1], 2);
                                let dRev = Math.pow(segCoords[segCoords.length - 1][0] - lastEndpoint[0], 2) + Math.pow(segCoords[segCoords.length - 1][1] - lastEndpoint[1], 2);
                                if (dRev < dFwd) segCoords.reverse();
                            }
                        }
                        lastEndpoint = segCoords[segCoords.length - 1];
                        scanLines.push(segCoords);
                    }
                }
                y += spacingDeg;
            }

            // Apply gimbal offset
            if (gimbalOffset > 0) {
                for (let i = 0; i < scanLines.length; i++) {
                    let lineCoords = scanLines[i];
                    if (lineCoords.length >= 2) {
                        let lineBrng = turf.rhumbBearing(lineCoords[0], lineCoords[lineCoords.length - 1]);
                        let offsetBrng = (-lineBrng + 360) % 360;
                        let offsetLine = [];
                        for (let pt of lineCoords) {
                            // turf rhumbDestination returns Feature
                            let dest = turf.rhumbDestination(pt, gimbalOffset, offsetBrng, { units: 'meters' });
                            offsetLine.push(dest.geometry.coordinates);
                        }
                        scanLines[i] = offsetLine;
                    }
                }
            }

            // Rotate back
            for (let lineCoords of scanLines) {
                let rotatedBack = rotateCoords(lineCoords, effectiveBearing, centroid);
                allFlightCoords.push(...rotatedBack);
            }
        }

        let flyingHeight = params.altitude - (params.takeOffOffset || 0);
        return { coords: allFlightCoords, photo_interval: photoInterval, flying_height: flyingHeight, gsd: gsd, line_spacing: lineSpacing, image_tag: isGrid ? "gridMap" : "map" };
    }

    function buildMapWaypoints(flightData, params) {
        const coords = flightData.coords;
        const photoInterval = flightData.photo_interval;
        const speed = params.speed || 10;
        const gimbalPitch = params.gimbalPitch !== undefined ? params.gimbalPitch : -90;
        let waypoints = [];
        let i = 0;
        while (i < coords.length - 1) {
            let start = coords[i], end = coords[i + 1];
            let yaw = roundAngle(bearingBetween(start, end));
            let segLength = distanceMeters(start, end);
            let action = { type: "SINGLE_PHOTO", yawAngle: yaw, gimbalPitchAngle: gimbalPitch };

            waypoints.push({ longitude: roundCoord(start[0]), latitude: roundCoord(start[1]), altitude: flightData.flying_height, actions: [action], speed, isFlyThrough: false, isFlightLineStart: true, isFlightLineEnd: false, imageTag: flightData.image_tag });

            if (segLength > 0 && photoInterval > 0) {
                let dist = photoInterval;
                while (dist < segLength) {
                    let pt = pointAlongLine(start, end, dist / segLength);
                    waypoints.push({ longitude: roundCoord(pt[0]), latitude: roundCoord(pt[1]), altitude: flightData.flying_height, actions: [action], speed, isFlyThrough: true, isFlightLineStart: false, isFlightLineEnd: false, imageTag: flightData.image_tag });
                    dist += photoInterval;
                }
            }
            waypoints.push({ longitude: roundCoord(end[0]), latitude: roundCoord(end[1]), altitude: flightData.flying_height, actions: [action], speed, isFlyThrough: false, isFlightLineStart: false, isFlightLineEnd: true, imageTag: flightData.image_tag });
            i += 2;
        }
        return waypoints;
    }

    function generateMapMission(hash, params, camera, isGrid = false) {
        let newParams = { ...params, isGridMap: isGrid };
        let flightData = generateMapPath(hash, newParams, camera);
        let waypoints = buildMapWaypoints(flightData, newParams);
        return { type: isGrid ? "gridMap" : "map", waypoints, pano_points: [], flight_coords: flightData.coords, photo_count: waypoints.length, gsd: flightData.gsd, line_spacing: flightData.line_spacing, photo_interval: flightData.photo_interval };
    }

    function generateHybridMission(hash, mapParams, panoParams, camera) {
        let mapMission = generateMapMission(hash, mapParams, camera);
        let panoPoints = getHexPanos(hash);
        let actions = getPanoActions(camera, panoParams);
        let panoWaypoints = panoPoints.map(pt => buildPanoWaypoint(pt[0], pt[1], panoParams.altitude, panoParams.speed, actions));
        return { type: "hybrid", waypoints: [...mapMission.waypoints, ...panoWaypoints], pano_points: panoPoints, flight_coords: mapMission.flight_coords, photo_count: mapMission.photo_count + (actions.length * panoPoints.length), gsd: mapMission.gsd, line_spacing: mapMission.line_spacing, photo_interval: mapMission.photo_interval };
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // KML GENERATOR
    // ─────────────────────────────────────────────────────────────────────────────
    function escapeXml(unsafe) {
        return unsafe.toString().replace(/[<>&'"]/g, function (c) {
            switch (c) { case '<': return '&lt;'; case '>': return '&gt;'; case '&': return '&amp;'; case '\'': return '&apos;'; case '"': return '&quot;'; }
        });
    }

    function buildKML(mission, hash, droneModel) {
        const hexCoords = getHexBoundary(hash);
        const centroid = getHexCentroid(hash);

        let lines = [];
        lines.push(`<?xml version="1.0" encoding="UTF-8"?>`);
        lines.push(`<kml xmlns="http://www.opengis.net/kml/2.2">`);
        lines.push(`  <Document>`);
        // mission.type is string like "map", "multiPanorama". Let's capitalize
        let missionName = mission.type.charAt(0).toUpperCase() + mission.type.slice(1);
        if (mission.type === "multiPanorama") missionName = "Multi-Panorama";
        if (mission.type === "gridMap") missionName = "Grid Map";

        lines.push(`    <name>Spexi ${missionName} Mission</name>`);

        // Styles
        // hexStyle: solid green border (ff alpha, 00=B, ff=G, 00=R), no fill
        lines.push(`    <Style id="hexStyle">
      <LineStyle><color>ff00ff00</color><width>3</width></LineStyle>
      <PolyStyle><fill>0</fill></PolyStyle>
    </Style>`);
        // pathStyle & transitStyle: solid pink line
        lines.push(`    <Style id="pathStyle">
      <LineStyle><color>ffff00ff</color><width>3</width></LineStyle>
    </Style>`);
        lines.push(`    <Style id="transitStyle">
      <LineStyle><color>ffff00ff</color><width>3</width></LineStyle>
    </Style>`);
        lines.push(`    <Style id="panoStyle">
      <IconStyle><color>ff00bbff</color><scale>1.2</scale><Icon><href>http://maps.google.com/mapfiles/kml/shapes/camera.png</href></Icon></IconStyle>
    </Style>`);
        lines.push(`    <Style id="panoPathStyle">
      <LineStyle><color>ffff00ff</color><width>2</width></LineStyle>
    </Style>`);

        // Helper to interpolate points for terrain following
        function interpolateLineStr(startPt, endPt, height) {
            let dist = distanceMeters(startPt, endPt);
            let segmentLength = 10; // 10 meters interpolation for smooth terrain curve
            let outCoords = [];

            if (dist <= segmentLength) {
                outCoords.push(`${roundCoord(startPt[0])},${roundCoord(startPt[1])},${height}`);
                outCoords.push(`${roundCoord(endPt[0])},${roundCoord(endPt[1])},${height}`);
                return outCoords.join(' ');
            }

            let segments = Math.ceil(dist / segmentLength);
            for (let j = 0; j <= segments; j++) {
                let frac = j / segments;
                let ipt = pointAlongLine(startPt, endPt, frac);
                outCoords.push(`${roundCoord(ipt[0])},${roundCoord(ipt[1])},${height}`);
            }
            return outCoords.join(' ');
        }

        // Hex Feature - Floating at flight altitude
        let hexAlt = mission.type.toLowerCase().includes("pano") ? PANO_PARAMS.altitude : MAP_PARAMS.altitude;

        // Output boundary with interpolation
        let hexCoordsInterp = [];
        for (let i = 0; i < hexCoords.length - 1; i++) {
            hexCoordsInterp.push(interpolateLineStr(hexCoords[i], hexCoords[i + 1], hexAlt).replace(/ [^ ]+$/, '')); // Avoid duplicate points
        }
        // close polygon smoothly
        hexCoordsInterp.push(`${roundCoord(hexCoords[hexCoords.length - 1][0])},${roundCoord(hexCoords[hexCoords.length - 1][1])},${hexAlt}`);
        let coordsStr = hexCoordsInterp.join(' ');

        lines.push(`    <Folder><name>Hex Boundary</name>
      <Placemark><name>Spexigon ${hash}</name><styleUrl>#hexStyle</styleUrl>
        <Polygon><altitudeMode>relativeToGround</altitudeMode>
          <tessellate>1</tessellate>
          <outerBoundaryIs><LinearRing><coordinates>${coordsStr}</coordinates></LinearRing></outerBoundaryIs>
        </Polygon>
      </Placemark></Folder>`);

        // Path Feature
        if (mission.flight_coords && mission.flight_coords.length >= 2) {
            let mapAlt = MAP_PARAMS.altitude;
            lines.push(`    <Folder><name>Flight Path</name>`);
            // Draw lines
            for (let i = 0; i < mission.flight_coords.length - 1; i += 2) {
                if (i + 1 < mission.flight_coords.length) {
                    let start = mission.flight_coords[i], end = mission.flight_coords[i + 1];
                    let lineStr = interpolateLineStr(start, end, mapAlt);
                    lines.push(`      <Placemark><name>Flight Line ${i / 2 + 1}</name><styleUrl>#pathStyle</styleUrl>
        <LineString><altitudeMode>relativeToGround</altitudeMode><tessellate>1</tessellate><coordinates>${lineStr}</coordinates></LineString></Placemark>`);
                }
            }
            // Draw transits
            for (let i = 1; i < mission.flight_coords.length - 1; i += 2) {
                if (i + 1 < mission.flight_coords.length) {
                    let start = mission.flight_coords[i], end = mission.flight_coords[i + 1];
                    let lineStr = interpolateLineStr(start, end, mapAlt);
                    lines.push(`      <Placemark><name>Transit ${Math.floor(i / 2) + 1}</name><styleUrl>#transitStyle</styleUrl>
        <LineString><altitudeMode>relativeToGround</altitudeMode><tessellate>1</tessellate><coordinates>${lineStr}</coordinates></LineString></Placemark>`);
                }
            }
            lines.push(`    </Folder>`);
        }

        if (mission.pano_points && mission.pano_points.length > 0) {
            lines.push(`    <Folder><name>Panorama Locations</name>`);
            let panoAlt = PANO_PARAMS.altitude;
            mission.pano_points.forEach((pt, idx) => {
                let name = (idx === 0 && mission.pano_points.length > 1) ? "Center Pano" : `Pano ${idx + 1}`;
                lines.push(`      <Placemark><name>${name}</name><styleUrl>#panoStyle</styleUrl>
        <Point><altitudeMode>relativeToGround</altitudeMode><coordinates>${pt[0]},${pt[1]},${panoAlt}</coordinates></Point></Placemark>`);
            });
            lines.push(`    </Folder>`);
        }

        lines.push(`  </Document>`);
        lines.push(`</kml>`);
        // Output with actual newlines to fix Google Earth Parse error
        return lines.join('\n');
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // UI Injection
    // ─────────────────────────────────────────────────────────────────────────────
    function injectUI() {
        const hashMatch = window.location.pathname.match(/\/zone\/([a-fA-F0-9]+)/);
        let h3Hash = hashMatch ? hashMatch[1] : null;

        // Try to find the button
        let buttons = Array.from(document.querySelectorAll("button"));
        let viewMapBtn = buttons.find(b => b.innerText && b.innerText.includes("View in Google Maps"));

        if (!viewMapBtn) return; // not found

        let container = viewMapBtn.parentElement;
        if (!container || document.getElementById('spexi-flight-btn-group')) return;

        h3Hash = h3Hash || document.querySelector("h1")?.innerText.split(" ")[0] || "8928d1a1a0bffff";

        // Create a wrapper
        let wrapper = document.createElement("div");
        wrapper.id = "spexi-flight-btn-group";
        wrapper.style.display = "inline-flex";
        wrapper.style.alignItems = "center";
        wrapper.style.position = "relative";

        let dlBtn = document.createElement("button");
        dlBtn.innerText = "Download Flight Path ▼";
        dlBtn.className = viewMapBtn.className;
        dlBtn.style.marginLeft = "8px";
        dlBtn.style.cursor = "pointer";

        let dropdown = document.createElement("div");
        dropdown.style.display = "none";
        dropdown.style.position = "absolute";
        dropdown.style.bottom = "100%";
        dropdown.style.right = "0";
        dropdown.style.backgroundColor = "#2f2f3e";
        dropdown.style.border = "1px solid #5a5a6b";
        dropdown.style.borderRadius = "6px";
        dropdown.style.padding = "4px 0";
        dropdown.style.minWidth = "180px";
        dropdown.style.zIndex = "999";
        dropdown.style.marginBottom = "4px";
        dropdown.style.boxShadow = "0 4px 12px rgba(0,0,0,0.4)";

        const options = [
            { id: 1, name: "Map" },
            { id: 2, name: "Multi-Panorama" },
            { id: 3, name: "Panorama" },
            { id: 4, name: "Grid Map" },
            { id: 5, name: "Hybrid" }
        ];

        options.forEach(opt => {
            let item = document.createElement("div");
            item.innerText = opt.name;
            item.style.padding = "8px 16px";
            item.style.color = "#fff";
            item.style.cursor = "pointer";
            item.style.fontFamily = "inherit";
            item.style.transition = "background-color 0.2s";

            item.onmouseover = () => item.style.backgroundColor = "#46465c";
            item.onmouseout = () => item.style.backgroundColor = "transparent";

            item.onclick = (e) => {
                e.stopPropagation();
                dropdown.style.display = "none";
                triggerDownload(opt.id, h3Hash);
            };
            dropdown.appendChild(item);
        });

        let divider = document.createElement("div");
        divider.style.height = "1px";
        divider.style.backgroundColor = "#5a5a6b";
        divider.style.margin = "4px 0";
        dropdown.appendChild(divider);

        let infoSection = document.createElement("div");
        infoSection.id = "spexi-drone-info-text";
        infoSection.style.padding = "4px 16px";
        infoSection.style.fontSize = "12px";
        infoSection.style.color = "#aaa";
        infoSection.innerText = `Drone: ${DRONE_MODEL}`;
        dropdown.appendChild(infoSection);

        let settingsItem = document.createElement("div");
        settingsItem.innerText = "⚙️ Settings";
        settingsItem.style.padding = "8px 16px";
        settingsItem.style.color = "#fff";
        settingsItem.style.cursor = "pointer";
        settingsItem.style.fontFamily = "inherit";
        settingsItem.style.fontSize = "13px";
        settingsItem.style.transition = "background-color 0.2s";

        settingsItem.onmouseover = () => settingsItem.style.backgroundColor = "#46465c";
        settingsItem.onmouseout = () => settingsItem.style.backgroundColor = "transparent";

        settingsItem.onclick = (e) => {
            e.stopPropagation();
            dropdown.style.display = "none";
            openSettingsModal();
        };
        dropdown.appendChild(settingsItem);

        wrapper.appendChild(dlBtn);
        wrapper.appendChild(dropdown);
        container.appendChild(wrapper);

        // Toggle dropdown
        dlBtn.onclick = (e) => {
            e.preventDefault();
            e.stopPropagation();
            if (dropdown.style.display === "none") {
                dropdown.style.display = "block";
            } else {
                dropdown.style.display = "none";
            }
        };

        // Close dropdown when clicking outside
        document.addEventListener("click", (e) => {
            if (!wrapper.contains(e.target)) dropdown.style.display = "none";
        });
    }

    function openSettingsModal() {
        if (document.getElementById("spexi-flight-settings-modal")) return;

        let overlay = document.createElement("div");
        overlay.id = "spexi-flight-settings-modal";
        overlay.style.position = "fixed";
        overlay.style.top = "0";
        overlay.style.left = "0";
        overlay.style.width = "100vw";
        overlay.style.height = "100vh";
        overlay.style.backgroundColor = "rgba(0, 0, 0, 0.7)";
        overlay.style.zIndex = "999999";
        overlay.style.display = "flex";
        overlay.style.justifyContent = "center";
        overlay.style.alignItems = "center";

        let modal = document.createElement("div");
        modal.style.backgroundColor = "#24252e";
        modal.style.borderRadius = "12px";
        modal.style.padding = "24px";
        modal.style.width = "380px";
        modal.style.maxWidth = "90%";
        modal.style.boxShadow = "0 8px 32px rgba(0,0,0,0.5)";
        modal.style.color = "#fff";
        modal.style.fontFamily = "inherit";
        modal.style.display = "flex";
        modal.style.flexDirection = "column";
        modal.style.gap = "16px";

        let title = document.createElement("h2");
        title.innerText = "Flight Planner Settings";
        title.style.margin = "0 0 8px 0";
        title.style.fontSize = "18px";
        title.style.borderBottom = "1px solid #3f3f4e";
        title.style.paddingBottom = "12px";
        modal.appendChild(title);

        let form = document.createElement("div");
        form.style.display = "flex";
        form.style.flexDirection = "column";
        form.style.gap = "12px";

        function createField(label, el) {
            let row = document.createElement("div");
            row.style.display = "flex";
            row.style.justifyContent = "space-between";
            row.style.alignItems = "center";

            let lbl = document.createElement("label");
            lbl.innerText = label;
            lbl.style.fontSize = "14px";
            lbl.style.color = "#ccc";

            row.appendChild(lbl);
            row.appendChild(el);
            return row;
        }

        let droneSel = document.createElement("select");
        droneSel.style.padding = "6px";
        droneSel.style.borderRadius = "4px";
        droneSel.style.backgroundColor = "#1a1a24";
        droneSel.style.color = "#fff";
        droneSel.style.border = "1px solid #3f3f4e";
        ["DJI Mini 2", "DJI Mini 3", "DJI Mini 3 Pro", "DJI Mini 4 Pro"].forEach(d => {
            let opt = document.createElement("option");
            opt.value = d;
            opt.innerText = d;
            if (d === DRONE_MODEL) opt.selected = true;
            droneSel.appendChild(opt);
        });
        form.appendChild(createField("Drone Model", droneSel));

        let tpTitle = document.createElement("h3");
        tpTitle.innerText = "Trusted Pilot Parameters";
        tpTitle.style.margin = "8px 0 0 0";
        tpTitle.style.fontSize = "14px";
        tpTitle.style.color = "#88d8b0";
        form.appendChild(tpTitle);

        function createSlider(id, min, max, step, val) {
            let wrap = document.createElement("div");
            wrap.style.display = "flex";
            wrap.style.alignItems = "center";
            wrap.style.gap = "8px";

            let input = document.createElement("input");
            input.type = "range";
            input.min = min;
            input.max = max;
            input.step = step;
            input.value = val;
            input.id = id;
            input.style.width = "100px";

            let disp = document.createElement("span");
            disp.innerText = val;
            disp.style.minWidth = "30px";
            disp.style.textAlign = "right";
            disp.style.fontSize = "13px";

            input.oninput = () => disp.innerText = input.value;

            wrap.appendChild(input);
            wrap.appendChild(disp);
            return { wrap, input };
        }

        let altUI = createSlider("altSlider", 80, 122, 2, MAP_PARAMS.altitude);
        form.appendChild(createField("Altitude (m)", altUI.wrap));

        let pitchUI = createSlider("pitchSlider", -90, -45, 2.5, MAP_PARAMS.gimbalPitch);
        form.appendChild(createField("Gimbal Pitch (°) (Map)", pitchUI.wrap));

        let pitchGridUI = createSlider("pitchGridSlider", -90, -45, 2.5, MAP_PARAMS.gimbalPitchGrid);
        form.appendChild(createField("Gimbal Pitch (°) (Grid Map)", pitchGridUI.wrap));

        let slapSel = document.createElement("select");
        slapSel.style.padding = "4px";
        slapSel.style.borderRadius = "4px";
        slapSel.style.backgroundColor = "#1a1a24";
        slapSel.style.color = "#fff";
        slapSel.style.border = "1px solid #3f3f4e";
        [0.70, 0.80].forEach(v => {
            let opt = document.createElement("option");
            opt.value = v;
            opt.innerText = (v * 100).toFixed(0) + "%";
            if (Math.abs(MAP_PARAMS.slap - v) < 0.01) opt.selected = true;
            slapSel.appendChild(opt);
        });
        form.appendChild(createField("Side Overlap", slapSel));

        modal.appendChild(form);

        let btnBox = document.createElement("div");
        btnBox.style.display = "flex";
        btnBox.style.justifyContent = "space-between";
        btnBox.style.marginTop = "8px";

        let leftBtns = document.createElement("div");
        leftBtns.style.display = "flex";
        leftBtns.style.gap = "8px";

        let closeBtn = document.createElement("button");
        closeBtn.innerText = "Cancel";
        closeBtn.style.padding = "8px 16px";
        closeBtn.style.backgroundColor = "transparent";
        closeBtn.style.color = "#ccc";
        closeBtn.style.border = "none";
        closeBtn.style.cursor = "pointer";
        closeBtn.style.fontFamily = "inherit";
        closeBtn.onclick = () => document.body.removeChild(overlay);

        let defaultBtn = document.createElement("button");
        defaultBtn.innerText = "Defaults";
        defaultBtn.style.padding = "8px 16px";
        defaultBtn.style.backgroundColor = "transparent";
        defaultBtn.style.color = "#ccc";
        defaultBtn.style.border = "1px solid #5a5a6b";
        defaultBtn.style.borderRadius = "4px";
        defaultBtn.style.cursor = "pointer";
        defaultBtn.style.fontFamily = "inherit";
        defaultBtn.onclick = () => {
            saveSettings({
                droneModel: droneSel.value,
                altitude: 80,
                gimbalPitch: -90,
                gimbalPitchGrid: -60,
                slap: 0.70
            });
            document.body.removeChild(overlay);
            openSettingsModal(); // Reload the modal to show defaults visually
        };

        leftBtns.appendChild(closeBtn);
        leftBtns.appendChild(defaultBtn);

        let saveBtn = document.createElement("button");
        saveBtn.innerText = "Save";
        saveBtn.style.padding = "8px 16px";
        saveBtn.style.backgroundColor = "#00ced1"; // Spexi-like cyan tone
        saveBtn.style.color = "#000";
        saveBtn.style.border = "none";
        saveBtn.style.borderRadius = "4px";
        saveBtn.style.cursor = "pointer";
        saveBtn.style.fontWeight = "bold";
        saveBtn.style.fontFamily = "inherit";

        saveBtn.onclick = () => {
            saveSettings({
                droneModel: droneSel.value,
                altitude: parseFloat(altUI.input.value),
                gimbalPitch: parseFloat(pitchUI.input.value),
                gimbalPitchGrid: parseFloat(pitchGridUI.input.value),
                slap: parseFloat(slapSel.value)
            });
            document.body.removeChild(overlay);
        };

        btnBox.appendChild(leftBtns);
        btnBox.appendChild(saveBtn);
        modal.appendChild(btnBox);
        overlay.appendChild(modal);
        document.body.appendChild(overlay);
    }

    function triggerDownload(type, cachedHash) {
        // Dynamically fetch the hash at click time so it stays accurate if the React DOM didn't reload the button
        let hash = window.location.pathname.match(/\/zone\/([a-fA-F0-9]+)/)?.[1] || document.querySelector("h1")?.innerText.split(" ")[0];
        if (!hash) {
            alert("Could not determine the current Hex Hash from the page.");
            return;
        }

        let camera = DRONES[DRONE_MODEL];
        let mission;
        console.log(`Generating mission type ${type} for hex ${hash}...`);

        try {
            if (type === 1) mission = generateMapMission(hash, MAP_PARAMS, camera, false);
            if (type === 2) mission = generateMultiPanoramaMission(hash, PANO_PARAMS, camera);
            if (type === 3) mission = generatePanoramaMission(hash, PANO_PARAMS, camera);
            if (type === 4) mission = generateMapMission(hash, { ...MAP_PARAMS, gimbalPitch: MAP_PARAMS.gimbalPitchGrid }, camera, true);
            if (type === 5) mission = generateHybridMission(hash, MAP_PARAMS, PANO_PARAMS, camera);
        } catch (e) {
            alert("Error generating flight path: " + e.message);
            console.error(e);
            return;
        }

        let kml = buildKML(mission, hash, DRONE_MODEL);

        let blob = new Blob([kml], { type: "application/vnd.google-earth.kml+xml" });
        let url = URL.createObjectURL(blob);
        let a = document.createElement("a");
        a.href = url;
        let typeName = FLIGHT_PLAN_NAMES[type].replace(/\s+/g, "");
        a.download = `Spexi_${typeName}_${hash}.kml`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
    }

    // Set up MutationObserver to repeatedly try injecting if React re-renders card
    const observer = new MutationObserver(() => injectUI());
    observer.observe(document.body, { childList: true, subtree: true });

    // Initial attempt
    window.addEventListener("load", () => {
        setTimeout(injectUI, 1000);
    });

})();
