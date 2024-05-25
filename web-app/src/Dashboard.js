// src/Dashboard.js
import React, { useEffect, useState } from 'react';
import { useTable, useSortBy } from 'react-table';

const ThingSpeakTable = () => {
    const [data, setData] = useState([]);

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
                    severity: feed.field6,
                }));
                setData(feeds);
            });
    }, []);

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

    const tableInstance = useTable({ columns, data }, useSortBy);

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
                        <tr {...row.getRowProps()}>
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

const Dashboard = () => {
    return (
        <div className="container">
            <aside>
                <h5>SideNav</h5>
            </aside>
            <main>
                <header>
                    <h2>Reports</h2>
                </header>
                <nav>
                    <h4>Finances</h4>
                </nav>
                <div className="grid">
                    <div className="grid-item big">
                        <div className="widget">
                            <h3>CASES</h3>
                            <ThingSpeakTable />
                        </div>
                    </div>
                    <div className="grid-item regular">3</div>
                    <div className="grid-item regular">5</div>
                    <div className="grid-item long">6</div>
                    <div className="grid-item regular">8</div>
                    <div className="grid-item tiny">9</div>
                </div>
            </main>
        </div>
    );
};

export default Dashboard;
