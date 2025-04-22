// ==UserScript==
// @name         Spexi Network Explorer 3D (Terrain) Mode
// @author       Secured_ on Discord
// @namespace    http://tampermonkey.net/
// @version      1.0
// @description  Button to toggle 3D (Terrain) View on the Spexi Network Explorer
// @match        https://explorer.spexi.com/*
// @license      MIT
// @grant        none
// @run-at       document-start
// ==/UserScript==

(function () {
    'use strict';

    let mapCounter = 0;

    // Intercept Mapbox Map instance for use when modifying the map, and re-add button when map reloads
    Object.defineProperty(Object.prototype, '_map', {
        configurable: true,
        set(val) {
            if (
                val &&
                typeof val.getStyle === 'function' &&
                typeof val.setTerrain === 'function' &&
                typeof val.addSource === 'function'
            ) {
                window.__speximap = val;
                mapCounter += 1;

                const thisMapId = mapCounter;

                setTimeout(() => {
                    if (mapCounter === thisMapId) {
                        injectSidebarToggle();
                    }
                }, 100);
            }

            Object.defineProperty(this, '_map', {
                value: val,
                writable: true,
                configurable: true
            });
        }
    });

    function injectSidebarToggle() {
        const outer = document.querySelector(
            '.PJLV.PJLV-ihFkYvu-css'
        );
        const inner = outer.querySelector(
            '.c-kiAJIg.c-kiAJIg-iTKOFX-dir-v.c-kiAJIg-fVlWzK-spacing-s'
        );

        if (inner.querySelector('[data-spexi-terrain-toggle]')) {
            return;
        }

        const btn = document.createElement('button');
        btn.className =
            'c-kSHLrh c-kSHLrh-eBJLUK-variant-neutral_solid c-kSHLrh-gAsrNz-size-m c-kSHLrh-fNianW-isCircle-true c-kSHLrh-kEvnuF-cv PJLV';
        btn.textContent = '3D';
        btn.dataset.spexiTerrainToggle = 'true';

        let terrainEnabled = false;

        btn.onclick = () => {
            const map = window.__speximap;
            if (!map) return console.error('(SpexiNetworkExplorerTerrainView) Error: Map not available');

            try {
                if (!terrainEnabled) {
                    if (!map.getSource('mapbox-dem')) {
                        map.addSource('mapbox-dem', {
                            type: 'raster-dem',
                            url: 'mapbox://mapbox.mapbox-terrain-dem-v1',
                            tileSize: 512,
                            maxzoom: 14
                        });
                    }
                    map.setTerrain({ source: 'mapbox-dem' });
                    btn.textContent = '2D';
                    terrainEnabled = true;
                } else {
                    map.setTerrain(null);
                    btn.textContent = '3D';
                    terrainEnabled = false;
                }
            } catch (err) {
                console.error('(SpexiNetworkExplorerTerrainView) Error: Terrain toggle error:', err);
            }
        };

        inner.insertBefore(btn, inner.firstChild);
    }
})();
