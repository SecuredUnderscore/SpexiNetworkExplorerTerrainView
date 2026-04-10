// ==UserScript==
// @name         Spexi Network Explorer Tools
// @author       Secured_ on Discord
// @namespace    http://tampermonkey.net/
// @version      3.5.0
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
            } catch (e) {
                // Silently skip failed intersections
            }
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
            let accumulatedCoords = []; // Flat array of endpoint coords, like app's _closure3_slot1
            let y = min_y + startOffset;

            while (y <= max_y) {
                let scanLineStart = [min_x - 0.001, y];
                let scanLineEnd = [max_x + 0.001, y];
                let scanLine = turf.lineString([scanLineStart, scanLineEnd]);
                // Cut line by polygon
                let segments = [];
                try {
                    let split = turf.lineSplit(scanLine, rotatedPoly);
                    if (!split || split.features.length === 0) {
                        let mid = turf.midpoint(scanLineStart, scanLineEnd);
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
                        // Step 1: Sort segment endpoints by distance from scan line start (DESCENDING)
                        // App comparator returns (dist_a1 - dist_a0), putting farthest-from-left first
                        let startRef = scanLineStart;
                        segCoords.sort((a, b) => {
                            let dA = distanceMeters(startRef, a);
                            let dB = distanceMeters(startRef, b);
                            return dB - dA;
                        });

                        // Step 2: Keep only the two endpoints (first and last)
                        // This matches the app's splice(1, length-2)
                        if (segCoords.length > 2) {
                            segCoords = [segCoords[0], segCoords[segCoords.length - 1]];
                        }

                        // Step 3: Direction logic based on accumulated state
                        if (accumulatedCoords.length > 0) {
                            if (gimbalPitch === -90) {
                                // Nadir: sort by proximity to last accumulated point
                                let lastPt = accumulatedCoords[accumulatedCoords.length - 1];
                                let d0 = distanceMeters(lastPt, segCoords[0]);
                                let d1 = distanceMeters(lastPt, segCoords[segCoords.length - 1]);
                                if (d1 < d0) segCoords.reverse();
                            } else {
                                // Non-nadir: alternate based on accumulated line count
                                let lineCount = accumulatedCoords.length / 2;
                                if (lineCount % 2 !== 0) segCoords.reverse();
                            }
                        }

                        accumulatedCoords.push(segCoords[0], segCoords[segCoords.length - 1]);
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
                    if ((segLength - dist) >= (photoInterval / 3)) {
                        let pt = pointAlongLine(start, end, dist / segLength);
                        waypoints.push({ longitude: roundCoord(pt[0]), latitude: roundCoord(pt[1]), altitude: flightData.flying_height, actions: [action], speed, isFlyThrough: true, isFlightLineStart: false, isFlightLineEnd: false, imageTag: flightData.image_tag });
                    }
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

        // Group waypoints into flight lines for KML rendering at real waypoint density
        let flightLineGroups = [];
        let currentLine = [];
        for (let wp of waypoints) {
            currentLine.push([wp.longitude, wp.latitude]);
            if (wp.isFlightLineEnd) {
                if (currentLine.length >= 2) flightLineGroups.push(currentLine);
                currentLine = [];
            }
        }
        if (currentLine.length >= 2) flightLineGroups.push(currentLine);

        return { type: isGrid ? "gridMap" : "map", waypoints, pano_points: [], flight_coords: flightData.coords, flight_lines: flightLineGroups, photo_count: waypoints.length, gsd: flightData.gsd, line_spacing: flightData.line_spacing, photo_interval: flightData.photo_interval };
    }

    // ── Hybrid: Snap & Interleave (matches app logic) ────────────────────────
    function snapAndInterleave(mapWaypoints, panoPoints, panoParams, camera) {
        let panoActions = getPanoActions(camera, panoParams);
        let usedIndices = new Set();
        let assignments = [];

        // Step 1: For each pano, find nearest unused map waypoint
        for (let pi = 0; pi < panoPoints.length; pi++) {
            let pano = panoPoints[pi];
            let bestDist = Infinity;
            let bestIdx = -1;
            for (let mi = 0; mi < mapWaypoints.length; mi++) {
                if (usedIndices.has(mi)) continue;
                let d = distanceMeters([mapWaypoints[mi].longitude, mapWaypoints[mi].latitude], pano);
                if (d < bestDist) { bestDist = d; bestIdx = mi; }
            }
            if (bestIdx >= 0) {
                usedIndices.add(bestIdx);
                assignments.push({ panoIdx: pi, mapIdx: bestIdx });
            }
        }

        // Step 2: Sort by map index so panos are visited in scan order
        assignments.sort((a, b) => a.mapIdx - b.mapIdx);

        // Step 3: Interleave — insert pano at matched map waypoint position
        let result = [];
        let cursor = 0;
        for (let a of assignments) {
            while (cursor < a.mapIdx) { result.push(mapWaypoints[cursor]); cursor++; }
            let mw = mapWaypoints[a.mapIdx];
            result.push(buildPanoWaypoint(mw.longitude, mw.latitude, panoParams.altitude, panoParams.speed, panoActions));
            cursor++; // skip past the replaced map waypoint
        }
        while (cursor < mapWaypoints.length) { result.push(mapWaypoints[cursor]); cursor++; }

        return result;
    }

    function generateHybridMission(hash, mapParams, panoParams, camera) {
        let mapMission = generateMapMission(hash, mapParams, camera);
        let panoPoints = getHexPanos(hash);
        let interleaved = snapAndInterleave(mapMission.waypoints, panoPoints, panoParams, camera);

        // Group into flight lines (same logic, skipping pano waypoints for line grouping)
        let flightLineGroups = [];
        let currentLine = [];
        for (let wp of interleaved) {
            if (wp.imageTag === "pano") continue;
            currentLine.push([wp.longitude, wp.latitude]);
            if (wp.isFlightLineEnd) {
                if (currentLine.length >= 2) flightLineGroups.push(currentLine);
                currentLine = [];
            }
        }
        if (currentLine.length >= 2) flightLineGroups.push(currentLine);

        let panoMarkers = interleaved.filter(wp => wp.imageTag === "pano").map(wp => [wp.longitude, wp.latitude]);
        let panoActions = getPanoActions(camera, panoParams);
        let photoCount = interleaved.reduce((sum, wp) => sum + (wp.actions ? wp.actions.length : 0), 0);

        return {
            type: "hybrid",
            waypoints: interleaved,
            pano_points: panoMarkers,
            flight_coords: mapMission.flight_coords,
            flight_lines: flightLineGroups,
            photo_count: photoCount,
            gsd: mapMission.gsd,
            line_spacing: mapMission.line_spacing,
            photo_interval: mapMission.photo_interval
        };
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

        lines.push(`    <name>Spexi ${hash}-${missionName}</name>`);

        // Extended Metadata
        lines.push(`    <ExtendedData>`);
        lines.push(`      <Data name="ImageCount"><value>${mission.photo_count || 0}</value></Data>`);
        if (droneModel) lines.push(`      <Data name="DroneModel"><value>${escapeXml(droneModel)}</value></Data>`);
        lines.push(`      <Data name="MissionType"><value>${missionName}</value></Data>`);
        lines.push(`      <Data name="ZoneHash"><value>${hash}</value></Data>`);
        lines.push(`    </ExtendedData>`);

        // Styles
        // hexStyle: solid green border (ff alpha, 00=B, ff=G, 00=R), no fill
        lines.push(`    <Style id="hexStyle">
      <LineStyle><color>ff00ff00</color><width>3</width></LineStyle>
      <PolyStyle><fill>0</fill></PolyStyle>
    </Style>`);
        // pathStyle: solid pink line (AABBGGRR: ff=Alpha, ff=Blue, 00=Green, ff=Red)
        lines.push(`    <Style id="pathStyle">
      <LineStyle><color>ffff00ff</color><width>3</width></LineStyle>
    </Style>`);

        // transitStyle: Orange for all transit lines (AABBGGRR: ff=Alpha, 00=Blue, a5=Green, ff=Red)
        lines.push(`    <Style id="transitStyle">
      <LineStyle><color>ff00a5ff</color><width>3</width></LineStyle>
    </Style>`);
        lines.push(`    <Style id="panoStyle">
      <IconStyle><color>ff00bbff</color><scale>1.2</scale><Icon><href>http://maps.google.com/mapfiles/kml/shapes/camera.png</href></Icon></IconStyle>
    </Style>`);
        lines.push(`    <Style id="imageStyle">
      <IconStyle><color>ff00ff00</color><scale>0.8</scale><Icon><href>http://maps.google.com/mapfiles/kml/shapes/camera.png</href></Icon></IconStyle>
    </Style>`);
        lines.push(`    <Style id="panoPathStyle">
      <LineStyle><color>ffff00ff</color><width>2</width></LineStyle>
    </Style>`);
        // Start/End point styles
        lines.push(`    <Style id="startStyle">
      <IconStyle><color>ff00ff00</color><scale>1.2</scale><Icon><href>http://maps.google.com/mapfiles/kml/paddle/grn-circle.png</href></Icon></IconStyle>
      <LabelStyle><scale>0.9</scale></LabelStyle>
    </Style>`);
        lines.push(`    <Style id="endStyle">
      <IconStyle><color>ff0000ff</color><scale>1.2</scale><Icon><href>http://maps.google.com/mapfiles/kml/paddle/red-circle.png</href></Icon></IconStyle>
      <LabelStyle><scale>0.9</scale></LabelStyle>
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

        lines.push(`    <Folder><name>Hex Boundary</name><visibility>0</visibility>
      <Placemark><name>Spexigon ${hash}</name><visibility>0</visibility><styleUrl>#hexStyle</styleUrl>
        <Polygon><altitudeMode>relativeToGround</altitudeMode>
          <tessellate>1</tessellate>
          <outerBoundaryIs><LinearRing><coordinates>${coordsStr}</coordinates></LinearRing></outerBoundaryIs>
        </Polygon>
      </Placemark></Folder>`);

        // Path Feature — uses real waypoint density when available
        if (mission.flight_lines && mission.flight_lines.length > 0) {
            let mapAlt = MAP_PARAMS.altitude;
            lines.push(`    <Folder><name>Flight Path</name>`);

            // Draw flight lines through all waypoint positions
            mission.flight_lines.forEach((line, i) => {
                let coordParts = [];
                for (let j = 0; j < line.length - 1; j++) {
                    coordParts.push(interpolateLineStr(line[j], line[j + 1], mapAlt));
                }
                let lineStr = coordParts.join(' ');
                lines.push(`      <Placemark><name>Flight Line ${i + 1}</name><styleUrl>#pathStyle</styleUrl>
        <LineString><altitudeMode>relativeToGround</altitudeMode><tessellate>1</tessellate><coordinates>${lineStr}</coordinates></LineString></Placemark>`);
            });

            // Draw transits between flight lines
            for (let i = 0; i < mission.flight_lines.length - 1; i++) {
                let prevLine = mission.flight_lines[i];
                let nextLine = mission.flight_lines[i + 1];
                let start = prevLine[prevLine.length - 1];
                let end = nextLine[0];
                let lineStr = interpolateLineStr(start, end, mapAlt);
                lines.push(`      <Placemark><name>Transit ${i + 1}</name><styleUrl>#transitStyle</styleUrl>
        <LineString><altitudeMode>relativeToGround</altitudeMode><tessellate>1</tessellate><coordinates>${lineStr}</coordinates></LineString></Placemark>`);
            }

            lines.push(`    </Folder>`);

            // Start and End points for map/gridMap/hybrid missions
            if (["map", "gridMap", "hybrid"].includes(mission.type)) {
                let firstLine = mission.flight_lines[0];
                let lastLine = mission.flight_lines[mission.flight_lines.length - 1];
                let startPt = firstLine[0];
                let endPt = lastLine[lastLine.length - 1];
                lines.push(`    <Folder><name>Start / End</name>`);
                lines.push(`      <Placemark><name>Start</name><styleUrl>#startStyle</styleUrl>
        <Point><altitudeMode>relativeToGround</altitudeMode><coordinates>${roundCoord(startPt[0])},${roundCoord(startPt[1])},${mapAlt}</coordinates></Point></Placemark>`);
                lines.push(`      <Placemark><name>End</name><styleUrl>#endStyle</styleUrl>
        <Point><altitudeMode>relativeToGround</altitudeMode><coordinates>${roundCoord(endPt[0])},${roundCoord(endPt[1])},${mapAlt}</coordinates></Point></Placemark>`);
                lines.push(`    </Folder>`);
            }
        } else if (mission.flight_coords && mission.flight_coords.length >= 2) {
            // Fallback: pair-based rendering (multi-panorama transit paths)
            let mapAlt = MAP_PARAMS.altitude;
            lines.push(`    <Folder><name>Flight Path</name>`);
            for (let i = 0; i < mission.flight_coords.length - 1; i++) {
                let start = mission.flight_coords[i], end = mission.flight_coords[i + 1];
                let lineStr = interpolateLineStr(start, end, mapAlt);
                lines.push(`      <Placemark><name>Transit ${i + 1}</name><styleUrl>#transitStyle</styleUrl>
        <LineString><altitudeMode>relativeToGround</altitudeMode><tessellate>1</tessellate><coordinates>${lineStr}</coordinates></LineString></Placemark>`);
            }
            lines.push(`    </Folder>`);
        }

        if (mission.pano_points && mission.pano_points.length > 0) {
            lines.push(`    <Folder><name>Panorama Locations</name>`);
            let panoAlt = PANO_PARAMS.altitude;
            mission.pano_points.forEach((pt, idx) => {
                let name = `Pano ${idx + 1}`;
                lines.push(`      <Placemark><name>${name}</name><styleUrl>#panoStyle</styleUrl>
        <Point><altitudeMode>relativeToGround</altitudeMode><coordinates>${pt[0]},${pt[1]},${panoAlt}</coordinates></Point></Placemark>`);
            });
            lines.push(`    </Folder>`);
        }

        let mapWaypoints = mission.waypoints ? mission.waypoints.filter(wp => wp.imageTag !== 'pano') : [];
        if (mapWaypoints.length > 0) {
            lines.push(`    <Folder><name>Image Locations</name><visibility>0</visibility>`);
            mapWaypoints.forEach((wp, idx) => {
                let name = `Image ${idx + 1}`;
                lines.push(`      <Placemark><name>${name}</name><visibility>0</visibility><styleUrl>#imageStyle</styleUrl>
        <Point><altitudeMode>relativeToGround</altitudeMode><coordinates>${wp.longitude},${wp.latitude},${wp.altitude}</coordinates></Point></Placemark>`);
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
        dlBtn.title = "Generate and download a KML flight path for Google Earth";

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

    // ─────────────────────────────────────────────────────────────────────────────
    // Available Missions (API)
    // ─────────────────────────────────────────────────────────────────────────────
    const API_BASE = "https://api.explorer.spexi.com/api";

    // ── USDC conversion rates (cached) ─────────────────────────────────────
    let _usdcCadRate = null;
    let _usdcCadRateTimestamp = 0;
    const RATE_CACHE_MS = 5 * 60 * 1000; // cache for 5 minutes

    async function fetchUsdcCadRate() {
        if (_usdcCadRate !== null && (Date.now() - _usdcCadRateTimestamp) < RATE_CACHE_MS) {
            return _usdcCadRate;
        }
        try {
            let resp = await fetch("https://api.coingecko.com/api/v3/simple/price?ids=usd-coin&vs_currencies=cad");
            if (!resp.ok) return _usdcCadRate; // return stale if available
            let json = await resp.json();
            if (json["usd-coin"] && json["usd-coin"].cad) {
                _usdcCadRate = json["usd-coin"].cad;
                _usdcCadRateTimestamp = Date.now();
            }
        } catch (e) {
            console.warn("[Spexi Tools] Failed to fetch USDC/CAD rate:", e);
        }
        return _usdcCadRate;
    }

    async function fetchMissions(hash) {
        try {
            let resp = await fetch(`${API_BASE}/missions?zone_hash=${hash}&status=active`, {
                credentials: "include",
                headers: { "Content-Type": "application/json" }
            });
            if (!resp.ok) return [];
            let json = await resp.json();
            return json.data || [];
        } catch (e) {
            return [];
        }
    }

    async function fetchFlights(hash) {
        try {
            let resp = await fetch(`${API_BASE}/flights?zone_hash=${hash}`, {
                credentials: "include",
                headers: { "Content-Type": "application/json" }
            });
            if (!resp.ok) return [];
            let json = await resp.json();
            return json.data || [];
        } catch (e) {
            return [];
        }
    }

    async function fetchZoneInfo(hash) {
        try {
            let resp = await fetch(`${API_BASE}/zones/${hash}`, {
                credentials: "include",
                headers: { "Content-Type": "application/json" }
            });
            if (!resp.ok) return null;
            let json = await resp.json();
            return json.data || null;
        } catch (e) {
            return null;
        }
    }

    function formatCurrency(amount, currency) {
        if (!amount || amount <= 0) return null;
        let sym = currency === "CAD" ? "CA$" : "$";
        return `${sym}${(amount / 100).toFixed(2)}`;
    }

    function groupMissions(missions) {
        let groups = {};
        for (let m of missions) {
            let fpId = m.flight_plan_id || m.flight_plan?.id;
            if (!groups[fpId]) {
                groups[fpId] = {
                    flight_plan_id: fpId,
                    flight_plan_name: m.flight_plan?.name || FLIGHT_PLAN_NAMES[fpId] || "Unknown",
                    count: 0,
                    totalAmount: 0,
                    totalRp: 0,
                    totalUsdc: 0,
                    currency: m.currency || "USD"
                };
            }
            groups[fpId].count++;
            groups[fpId].totalAmount += m.amount || 0;
            groups[fpId].totalRp += m.rp_amount || 0;
            if (m.token_amount) {
                groups[fpId].totalUsdc += Number(m.token_amount) / 1e6;
            }
        }
        return Object.values(groups);
    }

    let _missionFetching = false;
    let _flightsFetching = false;
    let _flightsCache = { hash: null, flights: [] };

    function formatFlightDate(dateStr) {
        let d = new Date(dateStr);
        let months = ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"];
        return `${months[d.getMonth()]} ${String(d.getDate()).padStart(2, '0')}, ${d.getFullYear()}`;
    }

    const COMPACT_ROW_HEIGHT = 50;

    function buildCompactFlightTR(flight, hash) {
        // Create a <tr> matching the existing flight history row structure
        let tr = document.createElement("tr");
        tr.className = "c-cHCiJL c-gvgkRI";
        tr.setAttribute("data-spexi-injected", "true");
        tr.style.height = COMPACT_ROW_HEIGHT + "px";
        tr.style.transform = "translateY(0px)";

        let td = document.createElement("td");
        td.className = "c-inedAR c-inedAR-cmVlgk-hAlign-left c-inedAR-goqZZF-vAlign-center";

        // Outer card — same classes as existing entries
        let card = document.createElement("div");
        card.className = "c-FNXti c-FNXti-hVCjZQ-background-darken c-FNXti-kEvnuF-padding-s";

        // Layout row: info left, status badge right — same as existing entries
        let row = document.createElement("div");
        row.className = "c-kiAJIg c-kiAJIg-JrrAq-align-start c-kiAJIg-knmidH-justify-spaced";

        // Info column — same classes as existing entries
        let info = document.createElement("div");
        info.className = "c-kiAJIg c-kiAJIg-iTKOFX-dir-v c-kiAJIg-fVlWzK-spacing-s c-kiAJIg-ibPNjhd-css";

        // Compact single line with date, pilot, and ID
        let datePilot = document.createElement("p");
        datePilot.className = "c-lbNOYO c-lbNOYO-kpetlT-variant-primary_body c-lbNOYO-dKdvLu-size-s c-lbNOYO-iqKmYR-weight-medium c-lbNOYO-ihTrFnY-css";
        let dateStr = formatFlightDate(flight.created_at);
        let pilot = flight.username || "Unknown";
        datePilot.textContent = `${dateStr}`;
        info.appendChild(datePilot);

        let pilotLine = document.createElement("p");
        pilotLine.className = "c-lbNOYO c-lbNOYO-deQLyJ-variant-secondary_body c-lbNOYO-fsvfVm-size-xs c-lbNOYO-ihTrFnY-css";
        pilotLine.textContent = `Pilot: ${pilot}  ·  ID: ${flight.id}`;
        info.appendChild(pilotLine);

        row.appendChild(info);

        // Status badge — uses same class as existing entries (c-bvgJZe)
        let statusBadge = document.createElement("p");
        statusBadge.className = "c-bvgJZe";
        statusBadge.textContent = flight.status.toLowerCase();
        statusBadge.style.color = "#ED3232";
        statusBadge.style.borderColor = "#ED3232";
        row.appendChild(statusBadge);

        card.appendChild(row);
        td.appendChild(card);
        tr.appendChild(td);
        return tr;
    }

    function parseRowDate(tr) {
        // Extract the date text from an existing flight history <tr>
        // The first <p> with the primary_body variant class contains the date like "Dec 20, 2025"
        let dateEl = tr.querySelector(".c-lbNOYO-kpetlT-variant-primary_body");
        if (dateEl) {
            let d = new Date(dateEl.textContent.trim());
            if (!isNaN(d.getTime())) return d;
        }
        return null;
    }

    function renderExtraFlightsDOM(extraFlights, hash) {
        // Remove any previously injected rows
        document.querySelectorAll('tr[data-spexi-injected]').forEach(el => el.remove());

        if (extraFlights.length === 0) return;

        // Find the Flight History table's <tbody>
        let historyHeader = null;
        let walker = document.createTreeWalker(document.body, NodeFilter.SHOW_TEXT, null, false);
        let textNode;
        while (textNode = walker.nextNode()) {
            if (textNode.textContent.trim() === "Flight History") {
                historyHeader = textNode.parentElement;
                break;
            }
        }

        if (!historyHeader) return;

        // Walk up to find the container with the table
        let historyContainer = historyHeader;
        for (let i = 0; i < 10 && historyContainer; i++) {
            if (historyContainer.querySelector && historyContainer.querySelector("table")) {
                break;
            }
            historyContainer = historyContainer.parentElement;
        }

        if (!historyContainer) return;

        let tbody = historyContainer.querySelector("tbody");
        if (!tbody) return;

        // Get existing rows and their dates
        let existingRows = Array.from(tbody.querySelectorAll("tr:not([data-spexi-injected])"));
        let existingDates = existingRows.map(tr => ({ tr, date: parseRowDate(tr) }));

        // Insert each extra flight in chronological order (descending — newest first)
        let addedHeight = 0;
        for (let flight of extraFlights) {
            let flightDate = new Date(flight.created_at);
            let newTR = buildCompactFlightTR(flight, hash);
            let inserted = false;

            // Find the correct position: insert before the first existing row with an older date
            for (let i = 0; i < existingDates.length; i++) {
                if (existingDates[i].date && flightDate > existingDates[i].date) {
                    tbody.insertBefore(newTR, existingDates[i].tr);
                    // Update the list so subsequent inserts are aware of this new row
                    existingDates.splice(i, 0, { tr: newTR, date: flightDate });
                    inserted = true;
                    break;
                }
            }

            if (!inserted) {
                // Flight is older than all existing rows — append at end
                tbody.appendChild(newTR);
                existingDates.push({ tr: newTR, date: flightDate });
            }
            addedHeight += COMPACT_ROW_HEIGHT;
        }

        // Adjust the table height container to accommodate the new rows
        let heightDiv = historyContainer.querySelector("div[style*='height:']") ||
                        historyContainer.querySelector("[data-radix-scroll-area-viewport] > div > div[style*='height']");
        if (heightDiv && heightDiv.style.height) {
            let currentHeight = parseInt(heightDiv.style.height, 10);
            if (!isNaN(currentHeight)) {
                heightDiv.style.height = (currentHeight + addedHeight) + "px";
            }
        }
    }

    function hasInjectedFlightRows() {
        return document.querySelector('tr[data-spexi-injected]') !== null;
    }

    function injectFailedTakenFlights(hash) {
        // Remove any previously injected rows
        document.querySelectorAll('tr[data-spexi-injected]').forEach(el => el.remove());
        if (_flightsFetching) return;

        // If we already have cached data for this hash, just render from cache
        if (_flightsCache.hash === hash && _flightsCache.flights.length > 0) {
            renderExtraFlightsDOM(_flightsCache.flights, hash);
            return;
        }

        _flightsFetching = true;
        fetchFlights(hash).then(flights => {
            _flightsFetching = false;
            let extraFlights = flights.filter(f => {
                let s = f.status?.toLowerCase();
                return s === "failed" || s === "taken";
            });

            // Sort by created_at descending (most recent first)
            extraFlights.sort((a, b) => new Date(b.created_at) - new Date(a.created_at));

            // Cache the filtered results
            _flightsCache = { hash, flights: extraFlights };

            renderExtraFlightsDOM(extraFlights, hash);
        }).catch(() => {
            _flightsFetching = false;
        });
    }

    function injectMissionList(hash) {
        let existing = document.getElementById("spexi-missions-section");
        if (existing) existing.remove();
        if (_missionFetching) return;
        _missionFetching = true;

        // Determine what data to fetch based on whether we know the country yet
        // Zone info may already be cached from the reservation timer
        let zoneInfoPromise;
        if (_reservationCache.hash === hash && _reservationCache.data) {
            zoneInfoPromise = Promise.resolve(_reservationCache.data);
        } else {
            zoneInfoPromise = fetchZoneInfo(hash).then(data => {
                if (data) _reservationCache = { hash, data, fetching: false };
                return data;
            });
        }

        // Fetch missions, zone info, and (conditionally) the CAD rate in parallel
        Promise.all([fetchMissions(hash), zoneInfoPromise]).then(([missions, zoneData]) => {
            let country = zoneData?.country || null;

            // Only fetch CAD rate if zone is in Canada
            let ratePromise = (country === "CA") ? fetchUsdcCadRate() : Promise.resolve(null);

            return ratePromise.then(usdcCadRate => {
                _missionFetching = false;
                // Remove again in case one snuck in while we were fetching
                let dup = document.getElementById("spexi-missions-section");
                if (dup) dup.remove();
                // Build the section using the site's own Stitches CSS classes for consistent styling
                let section = document.createElement("div");
                section.id = "spexi-missions-section";
                section.className = "c-FNXti c-FNXti-hVCjZQ-background-darken c-FNXti-ivYAPe-padding-none c-FNXti-iydAuT-inlaid-true";
                section.style.padding = "12px 16px";

                // Header row: title + badge
                let header = document.createElement("div");
                header.className = "c-kiAJIg c-kiAJIg-knmidH-justify-spaced c-kiAJIg-eKWVTQ-spacing-m";
                header.style.marginBottom = "8px";
                header.style.display = "flex";
                header.style.alignItems = "center";

                let titleP = document.createElement("p");
                titleP.className = "c-lbNOYO c-lbNOYO-jwYGDW-variant-primary_header";
                let titleSpan = document.createElement("span");
                titleSpan.className = "c-jgPCyX c-jgPCyX-eKvWLo-variant-subtle";
                titleSpan.textContent = "Available Missions";
                titleP.appendChild(titleSpan);
                header.appendChild(titleP);

                let badge = document.createElement("span");
                badge.style.fontSize = "11px";
                badge.style.padding = "2px 8px";
                badge.style.borderRadius = "10px";
                badge.style.backgroundColor = missions.length > 0 ? "#1a6b3a" : "#5a5a6b";
                badge.style.color = "#fff";
                badge.style.flexShrink = "0";
                badge.textContent = missions.length > 0 ? `${missions.length} active` : "None";
                header.appendChild(badge);

                section.appendChild(header);

                // Show conversion rate info if applicable
                if (missions.length > 0) {
                    if (country === "CA" && usdcCadRate) {
                        let rateInfo = document.createElement("p");
                        rateInfo.style.fontSize = "11px";
                        rateInfo.style.color = "#888";
                        rateInfo.style.marginBottom = "4px";
                        rateInfo.innerHTML = `1 USDC \u2248 <span style="color:rgb(255,204,0)">CA$${usdcCadRate.toFixed(4)}</span>`;
                        section.appendChild(rateInfo);
                    } else if (country === "US") {
                        let rateInfo = document.createElement("p");
                        rateInfo.style.fontSize = "11px";
                        rateInfo.style.color = "#888";
                        rateInfo.style.marginBottom = "4px";
                        rateInfo.innerHTML = `1 USDC \u2248 <span style="color:rgb(100,200,100)">$1.00 USD</span>`;
                        section.appendChild(rateInfo);
                    }
                }

                if (missions.length === 0) {
                    let empty = document.createElement("p");
                    empty.className = "c-lbNOYO c-lbNOYO-deQLyJ-variant-secondary_body c-lbNOYO-fsvfVm-size-xs";
                    empty.textContent = "No active missions for this zone.";
                    section.appendChild(empty);
                } else {
                    let grouped = groupMissions(missions);
                    grouped.forEach(g => {
                        let row = document.createElement("div");
                        row.className = "c-kiAJIg c-kiAJIg-knmidH-justify-spaced";
                        row.style.display = "flex";
                        row.style.alignItems = "center";
                        row.style.padding = "12px 0";
                        row.style.gap = "14px";
                        row.style.borderBottom = "1px solid rgba(255,255,255,0.06)";

                        let info = document.createElement("div");
                        info.className = "c-kiAJIg c-kiAJIg-iTKOFX-dir-v c-kiAJIg-kdofoX-spacing-xs";

                        let nameEl = document.createElement("p");
                        nameEl.className = "c-lbNOYO c-lbNOYO-kpetlT-variant-primary_body c-lbNOYO-dKdvLu-size-s c-lbNOYO-iqKmYR-weight-medium";
                        let countLabel = g.count > 1 ? ` (\u00d7${g.count})` : "";
                        nameEl.textContent = `${g.flight_plan_name}${countLabel}`;
                        info.appendChild(nameEl);

                        let rewardSpans = [];
                        let cashStr = formatCurrency(g.totalAmount, g.currency);
                        if (cashStr) rewardSpans.push(`<span>${cashStr}</span>`);
                        if (g.totalUsdc > 0) {
                            let usdcStr = g.totalUsdc.toFixed(2);
                            rewardSpans.push(`<span>+${usdcStr} USDC</span>`);
                            if (country === "CA" && usdcCadRate) {
                                let cadEquiv = (g.totalUsdc * usdcCadRate).toFixed(2);
                                rewardSpans.push(`<span style="color:rgb(255,204,0)">\u2248 CA$${cadEquiv}</span>`);
                            } else if (country === "US") {
                                let usdEquiv = g.totalUsdc.toFixed(2);
                                rewardSpans.push(`<span style="color:rgb(100,200,100)">\u2248 $${usdEquiv} USD</span>`);
                            }
                        }
                        if (g.totalRp > 0) rewardSpans.push(`<span style="color:#fff">+${g.totalRp} RP</span>`);

                        if (rewardSpans.length > 0) {
                            let rewardEl = document.createElement("p");
                            rewardEl.className = "c-lbNOYO c-lbNOYO-deQLyJ-variant-secondary_body c-lbNOYO-fsvfVm-size-xs c-lbNOYO-iqKmYR-weight-medium";
                            rewardEl.style.color = "#88d8b0";
                            rewardEl.innerHTML = rewardSpans.join("&nbsp;&nbsp;");
                            info.appendChild(rewardEl);
                        }

                    row.appendChild(info);

                    let dlBtn = document.createElement("button");
                    dlBtn.className = "c-kSHLrh c-kSHLrh-imStlG-variant-neutral_text c-kSHLrh-fyQYCy-size-s PJLV";
                    dlBtn.style.flexShrink = "0";
                    dlBtn.textContent = "📥 KML";
                    dlBtn.title = "Download KML flight path for this mission type";

                    dlBtn.onclick = (e) => {
                        e.stopPropagation();
                        triggerDownload(g.flight_plan_id, hash);
                    };

                    row.appendChild(dlBtn);
                    section.appendChild(row);
                });
            }

            // Strategy 1: Find exact "24 hr Weather" leaf text element using TreeWalker
            let inserted = false;
            let walker = document.createTreeWalker(document.body, NodeFilter.SHOW_TEXT, null, false);
            let textNode;
            while (textNode = walker.nextNode()) {
                if (textNode.textContent.trim() === "24 hr Weather") {
                    // Walk up from the text node to find the outermost weather section container
                    let el = textNode.parentElement;
                    let candidate = el;
                    // Keep walking up until we find a container whose parent holds multiple top-level sections
                    // (the scrollable area that contains weather, flight history, etc.)
                    for (let i = 0; i < 8 && el; i++) {
                        if (el.parentElement && el.parentElement.children.length > 2) {
                            // This parent has many children — el is the weather section block
                            candidate = el;
                            break;
                        }
                        candidate = el;
                        el = el.parentElement;
                    }
                    candidate.parentElement.insertBefore(section, candidate);
                    inserted = true;
                    break;
                }
            }

            // Strategy 2: Fallback — find "View in Google Maps" button and insert before its container
            if (!inserted) {
                let buttons = Array.from(document.querySelectorAll("button"));
                let gmapBtn = buttons.find(b => b.innerText && b.innerText.includes("View in Google Maps"));
                if (gmapBtn) {
                    let container = gmapBtn.parentElement;
                    if (container) {
                        container.insertBefore(section, gmapBtn);
                        inserted = true;
                    }
                }
            }

            // Strategy 3: Last resort — find scrollable container via attribute
            if (!inserted) {
                let scrollable = document.querySelector("[scrollable='true']") || document.querySelector("[data-scrollable]");
                if (scrollable) {
                    scrollable.prepend(section);
                    inserted = true;
                }
            }

            if (!inserted) {
                // Silently fail if insertion points aren't found
            }

            // Also inject Failed/Taken flights into the Flight History section
            injectFailedTakenFlights(hash);
            }); // end ratePromise.then
        });
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Reservation Timer
    // ─────────────────────────────────────────────────────────────────────────────
    const RESERVATION_DURATION_MS = 48 * 60 * 60 * 1000; // 48 hours
    let _reservationTimerInterval = null;
    let _reservationCache = { hash: null, data: null, fetching: false };

    function formatCountdown(ms) {
        if (ms <= 0) return "0s";
        let totalSec = Math.floor(ms / 1000);
        let h = Math.floor(totalSec / 3600);
        let m = Math.floor((totalSec % 3600) / 60);
        let s = totalSec % 60;
        if (h > 0) return `${h}h ${String(m).padStart(2, '0')}m ${String(s).padStart(2, '0')}s`;
        if (m > 0) return `${m}m ${String(s).padStart(2, '0')}s`;
        return `${s}s`;
    }

    function clearReservationTimer() {
        if (_reservationTimerInterval) {
            clearInterval(_reservationTimerInterval);
            _reservationTimerInterval = null;
        }
        let existing = document.getElementById("spexi-reservation-timer");
        if (existing) existing.remove();
    }

    function injectReservationTimer(hash) {
        // Don't inject if there's already a native Reserve button
        let buttons = Array.from(document.querySelectorAll("button"));
        let reserveBtn = buttons.find(b => b.innerText && b.innerText.trim() === "Reserve Zone");
        if (reserveBtn) return;

        // Don't inject if timer already exists
        if (document.getElementById("spexi-reservation-timer")) return;

        // Guard: don't re-fetch if already fetching
        if (_reservationCache.fetching) return;

        // Use cached data if available for this hash
        if (_reservationCache.hash === hash && _reservationCache.data !== null) {
            renderReservationTimerFromData(_reservationCache.data, hash);
            return;
        }

        _reservationCache.fetching = true;
        fetchZoneInfo(hash).then(zoneData => {
            _reservationCache = { hash, data: zoneData, fetching: false };
            renderReservationTimerFromData(zoneData, hash);
        }).catch(() => {
            _reservationCache.fetching = false;
        });
    }

    function renderReservationTimerFromData(zoneData, hash) {
        if (!zoneData || !zoneData.reservation) return;
        let reservation = zoneData.reservation;
        if (reservation.status !== "active") return;

        let createdAt = new Date(reservation.created_at).getTime();
        let expiresAt = createdAt + RESERVATION_DURATION_MS;
        let now = Date.now();

        // If already expired, no timer needed
        if (now >= expiresAt) return;

        // Don't inject if native Reserve button appeared
        let buttons = Array.from(document.querySelectorAll("button"));
        let reserveBtn = buttons.find(b => b.innerText && b.innerText.trim() === "Reserve Zone");
        if (reserveBtn) return;

        // Don't duplicate
        if (document.getElementById("spexi-reservation-timer")) return;

        // Find the bottom bar container (where Google Maps and Download buttons are)
        let gmapBtn = buttons.find(b => b.innerText && b.innerText.includes("View in Google Maps"));
        if (!gmapBtn) return;

        let bottomBar = gmapBtn.parentElement;
        if (!bottomBar) return;

        // Create the timer button — same classes as the native Reserve Zone button
        let timerBtn = document.createElement("button");
        timerBtn.id = "spexi-reservation-timer";
        timerBtn.className = "c-kSHLrh c-kSHLrh-dXpgym-variant-primary_outline c-kSHLrh-fyQYCy-size-s PJLV";
        timerBtn.type = "button";
        timerBtn.style.cursor = "default";
        timerBtn.style.opacity = "0.7";
        timerBtn.title = "This button will become available when the current reservation expires";

        let innerDiv = document.createElement("div");
        innerDiv.className = "c-kiAJIg c-kiAJIg-jroWjL-align-center c-kiAJIg-bICGYT-justify-center c-kiAJIg-ejCoEP-dir-h c-kiAJIg-kdofoX-spacing-xs";

        let remaining = expiresAt - Date.now();
        innerDiv.textContent = `⏳ ${formatCountdown(remaining)}`;

        timerBtn.appendChild(innerDiv);
        bottomBar.appendChild(timerBtn);

        // Clear any existing interval
        if (_reservationTimerInterval) clearInterval(_reservationTimerInterval);

        // Start countdown
        _reservationTimerInterval = setInterval(() => {
            let remaining = expiresAt - Date.now();
            if (remaining <= 0) {
                clearInterval(_reservationTimerInterval);
                _reservationTimerInterval = null;
                // Timer expired — reload so the native Reserve button appears
                innerDiv.textContent = "Reserve Zone";
                timerBtn.style.cursor = "pointer";
                timerBtn.style.opacity = "1";
                timerBtn.title = "Reservation expired — click to refresh";
                timerBtn.onclick = () => location.reload();
            } else {
                innerDiv.textContent = `⏳ ${formatCountdown(remaining)}`;
            }
        }, 1000);
    }

    function checkReservationTimerReinjection() {
        let hash = getCurrentZoneHash();
        if (!hash) return;
        // Only re-inject if we have cached data and the timer element is missing
        if (_reservationCache.hash === hash && _reservationCache.data && !document.getElementById("spexi-reservation-timer")) {
            // Check that the bottom bar exists
            let buttons = Array.from(document.querySelectorAll("button"));
            let gmapBtn = buttons.find(b => b.innerText && b.innerText.includes("View in Google Maps"));
            if (gmapBtn) {
                renderReservationTimerFromData(_reservationCache.data, hash);
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Zone Change Detection & Initialization
    // ─────────────────────────────────────────────────────────────────────────────
    let lastZoneHash = null;

    function getCurrentZoneHash() {
        let m = window.location.pathname.match(/\/zone\/([a-fA-F0-9]+)/);
        return m ? m[1] : null;
    }

    function onZoneChange(hash) {
        if (!hash) return;
        lastZoneHash = hash;
        _flightsCache = { hash: null, flights: [] }; // invalidate cache for new zone
        clearReservationTimer();
        _reservationCache = { hash: null, data: null, fetching: false }; // invalidate reservation cache
        injectMissionList(hash);
        injectReservationTimer(hash);
    }

    function checkForExtraFlightsReinjection() {
        let hash = getCurrentZoneHash();
        if (hash && _flightsCache.hash === hash && _flightsCache.flights.length > 0 && !hasInjectedFlightRows()) {
            // Only re-render from cache — never re-fetch from the observer
            let walker = document.createTreeWalker(document.body, NodeFilter.SHOW_TEXT, null, false);
            let textNode;
            while (textNode = walker.nextNode()) {
                if (textNode.textContent.trim() === "Flight History") {
                    renderExtraFlightsDOM(_flightsCache.flights, hash);
                    return;
                }
            }
        }
    }

    function checkForZoneChange() {
        let hash = getCurrentZoneHash();
        if (hash && hash !== lastZoneHash) {
            onZoneChange(hash);
        } else if (hash && hash === lastZoneHash && !document.getElementById("spexi-missions-section")) {
            // React re-rendered and removed our section — re-inject
            injectMissionList(hash);
        }
    }

    // MutationObserver for UI injection + zone change detection
    const observer = new MutationObserver(() => {
        injectUI();
        checkForZoneChange();
        checkForExtraFlightsReinjection();
        checkReservationTimerReinjection();
    });

    function startObserving() {
        if (document.body) {
            observer.observe(document.body, { childList: true, subtree: true });
            setTimeout(() => {
                injectUI();
                checkForZoneChange();
            }, 1000);
        } else {
            document.addEventListener("DOMContentLoaded", () => {
                observer.observe(document.body, { childList: true, subtree: true });
                setTimeout(() => {
                    injectUI();
                    checkForZoneChange();
                }, 1000);
            });
        }
    }

    // Also watch for pushState/popState URL changes (SPA navigation)
    let origPushState = history.pushState;
    history.pushState = function () {
        origPushState.apply(this, arguments);
        setTimeout(checkForZoneChange, 300);
    };
    window.addEventListener("popstate", () => setTimeout(checkForZoneChange, 300));

    startObserving();

})();
