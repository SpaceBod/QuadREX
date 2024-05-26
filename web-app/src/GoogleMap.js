// src/GoogleMap.js
import React, { useEffect, useRef, useState } from 'react';

const GoogleMap = ({ markersData, selectedEntry }) => {
  const mapRef = useRef(null);
  const [map, setMap] = useState(null);
  const markersRef = useRef([]);

  const loadLibraries = async () => {
    if (!window.google) return;

    const { AdvancedMarkerElement, PinElement } = await window.google.maps.importLibrary('marker');
    const { Map } = await window.google.maps.importLibrary('maps');
    
    return { AdvancedMarkerElement, PinElement, Map };
  };

  useEffect(() => {
    const initializeMap = async () => {
      const libraries = await loadLibraries();
      if (!libraries) return;

      const { AdvancedMarkerElement, PinElement, Map } = libraries;

      if (mapRef.current && !map) {
        const mapInstance = new Map(mapRef.current, {
          center: { lat: 51.499185, lng: -0.172816 },
          zoom: 15,
          mapId: 'YOUR_MAP_ID' // Ensure to add your map ID here
        });
        setMap(mapInstance);
      }

      const getPinColor = (severity) => {
        if (severity >= 1 && severity <= 3) return '#00FF00'; // Green
        if (severity >= 4 && severity <= 6) return '#FFA500'; // Orange
        if (severity >= 7 && severity <= 10) return '#FF0000'; // Red
        return '#FFFFFF'; // Default color (white)
      };

      if (map) {
        // Clear existing markers
        markersRef.current.forEach(markerObj => markerObj.marker.setMap(null));
        markersRef.current = [];

        const newMarkers = markersData.map(data => {
          const { lat, lng } = data.coordinates; // Use stored coordinates

          const pinColor = getPinColor(data.severity);

          const pinElement = new PinElement({
            background: pinColor,
          });

          const marker = new AdvancedMarkerElement({
            position: { lat, lng },
            map,
            content: pinElement.element,
            title: `${data.name}\n${data.type}`,
          });

          return { marker, data, position: { lat, lng } };
        });

        markersRef.current = newMarkers;
      }

      if (selectedEntry && map) {
        const selectedMarker = markersRef.current.find(
          m => m.data.name === selectedEntry.name && m.data.type === selectedEntry.type
        );

        if (selectedMarker) {
          map.setZoom(18);
          map.setCenter(selectedMarker.position);

          const pinElement = new PinElement({
            background: '#00FFFF', // Blue color for selected marker
            borderColor: '#0000FF',
            glyphColor: '#0000FF',
            scale: 1, // Highlight the selected marker by scaling it
          });

          selectedMarker.marker.content = pinElement.element;

          setTimeout(() => {
            selectedMarker.marker.content = new PinElement({
              background: getPinColor(selectedMarker.data.severity),
            }).element; // Reset to original color after 2 seconds
          }, 2000);
        }
      }
    };

    initializeMap();
  }, [mapRef, map, markersData, selectedEntry]);

  return <div ref={mapRef} className="map-container"></div>;
};

export default GoogleMap;
