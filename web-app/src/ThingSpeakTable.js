// src/ThingSpeakTable.js
import React, { useEffect, useState } from 'react';
import { useTable, useSortBy } from 'react-table';

const ThingSpeakTable = ({ setData, setSelectedEntry }) => {
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

  useEffect(() => {
    fetch('https://api.thingspeak.com/channels/2561068/feeds.json?api_key=OE9W4WOACTCIH7U8&results=10')
      .then(response => response.json())
      .then(data => {
        const feeds = data.feeds.map(feed => ({
          time: feed.created_at,
          name: feed.field2,
          age: feed.field3,
          type: feed.field4,
          details: feed.field5,
          severity: parseInt(feed.field6, 10),
          coordinates: generateCoordinates(), // Generate coordinates once
        }));
        setLocalData(feeds);
        setData(feeds); // Pass data to Dashboard
      });
  }, [setData]);

  const columns = React.useMemo(
    () => [
      { Header: 'Time', accessor: 'time' },
      { Header: 'Name', accessor: 'name' },
      { Header: 'Age', accessor: 'age' },
      { Header: 'Type', accessor: 'type' },
      { Header: 'Details', accessor: 'details' },
      { Header: 'Severity', accessor: 'severity' },
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
        {headerGroups.map(headerGroup => (
          <tr {...headerGroup.getHeaderGroupProps()}>
            {headerGroup.headers.map(column => (
              <th {...column.getHeaderProps(column.getSortByToggleProps())}>
                {column.render('Header')}
                <span>
                  {column.isSorted
                    ? column.isSortedDesc
                      ? ' 🔽'
                      : ' 🔼'
                    : ''}
                </span>
              </th>
            ))}
          </tr>
        ))}
      </thead>
      <tbody {...getTableBodyProps()}>
        {rows.map(row => {
          prepareRow(row);
          return (
            <tr
              {...row.getRowProps()}
              onClick={() => setSelectedEntry(row.original)} // Handle row click
              style={{ cursor: 'pointer' }}
            >
              {row.cells.map(cell => (
                <td {...cell.getCellProps()}>{cell.render('Cell')}</td>
              ))}
            </tr>
          );
        })}
      </tbody>
    </table>
  );
};

export default ThingSpeakTable;
