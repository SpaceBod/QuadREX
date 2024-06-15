import React, { useEffect, useRef, useState } from 'react';

const RobotMap = ({ markersData, selectedEntry }) => {
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

      if (map) {
        // Clear existing markers
        markersRef.current.forEach(markerObj => markerObj.marker.setMap(null));
        markersRef.current = [];

        const newMarkers = markersData.map(data => {
          const { lat, lng } = data.coordinates; // Use stored coordinates

          const marker = new AdvancedMarkerElement({
            position: { lat, lng },
            map,
            title: `${data.name}\n${data.ip}`,
          });

          return { marker, data, position: { lat, lng } };
        });

        markersRef.current = newMarkers;
      }

      if (selectedEntry && map) {
        const selectedMarker = markersRef.current.find(
          m => m.data.name === selectedEntry.name && m.data.ip === selectedEntry.ip
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
              background: '#FF0000', // Reset to default color
            }).element; // Reset to original color after 2 seconds
          }, 2000);
        }
      }
    };

    initializeMap();
  }, [mapRef, map, markersData, selectedEntry]);

  return <div ref={mapRef} className="map-container"></div>;
};

export default RobotMap;
