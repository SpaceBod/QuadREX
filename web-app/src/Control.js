import React, { useEffect, useState } from 'react';
import { useTable, useSortBy } from 'react-table';

const Control = ({ setData, setSelectedEntry }) => {
  const [localData, setLocalData] = useState([]);

  const generateCoordinates = () => {
    const bounds = {
      north: 51.50138311851994,
      south: 51.496988252410425,
      east: -0.16535386212148495,
      west: -0.18027906900681118,
    };

    const lat = bounds.south + Math.random() * (bounds.north - bounds.south);
    const lng = bounds.west + Math.random() * (bounds.east - bounds.west);

    return { lat, lng };
  };

  const generateRandomNumber = (min, max) => {
    return Math.floor(Math.random() * (max - min + 1)) + min;
  };

  useEffect(() => {
    fetch('https://api.thingspeak.com/channels/2530164/feeds.json?api_key=5EZQZMTCQ3EEU314&results=10')
      .then(response => response.json())
      .then(data => {
        const feeds = data.feeds.map((feed, index) => ({
          id: generateRandomNumber(100, 999),
          name: `REX-${String(index + 1).padStart(2, '0')}`,
          date: feed.field1,
          ip: feed.field2,
          temperature: generateRandomNumber(14, 20),
          coordinates: generateCoordinates(),
        }));
        console.log('Fetched Data:', feeds); // Log the fetched data
        setLocalData(feeds);
        setData(feeds); // Pass data to Dashboard
      })
      .catch(error => console.error('Error fetching data:', error));
  }, [setData]);

  useEffect(() => {
    console.log('Local Data:', localData); // Log localData after it's set
  }, [localData]);

  const columns = React.useMemo(
    () => [
      { Header: 'ID', accessor: 'id' },
      { Header: 'Name', accessor: 'name' },
      { Header: 'Location', accessor: row => `${row.coordinates?.lat.toFixed(5)}, ${row.coordinates?.lng.toFixed(5)}`, id: 'location' },
      { Header: 'IP Address', accessor: 'ip' },
    ],
    []
  );

  const tableInstance = useTable({ columns, data: localData }, useSortBy);

  const {
    getTableProps,
    getTableBodyProps,
    headerGroups,
    rows,
    prepareRow,
  } = tableInstance;

  return (
    <table {...getTableProps()} className="table">
      <thead>
        {headerGroups.map(headerGroup => {
          const { key, ...headerGroupProps } = headerGroup.getHeaderGroupProps(); // Destructure to separate key from other props
          return (
            <tr key={key} {...headerGroupProps}>
              {headerGroup.headers.map(column => {
                const { key: columnKey, ...columnProps } = column.getHeaderProps(column.getSortByToggleProps()); // Destructure to separate key from other props
                return (
                  <th key={columnKey} {...columnProps}>
                    {column.render('Header')}
                    <span>
                      {column.isSorted
                        ? column.isSortedDesc
                          ? ' 🔽'
                          : ' 🔼'
                        : ''}
                    </span>
                  </th>
                );
              })}
            </tr>
          );
        })}
      </thead>
      <tbody {...getTableBodyProps()}>
        {rows.map(row => {
          prepareRow(row);
          const { key, ...rowProps } = row.getRowProps(); // Destructure to separate key from other props
          return (
            <tr
              key={key} // Explicitly set the key prop
              {...rowProps}
              onClick={() => setSelectedEntry(row.original)} // Handle row click
              style={{ cursor: 'pointer' }}
            >
              {row.cells.map(cell => {
                const { key: cellKey, ...cellProps } = cell.getCellProps(); // Destructure to separate key from other props
                return (
                  <td key={cellKey} {...cellProps}>{cell.render('Cell')}</td>
                );
              })}
            </tr>
          );
        })}
      </tbody>
    </table>
  );
};

export default Control;
