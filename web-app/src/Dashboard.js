// src/Dashboard.js
import React, { useState } from 'react';
import ThingSpeakTable from './ThingSpeakTable';
import GoogleMap from './GoogleMap'; // Import the GoogleMap component

const Dashboard = () => {
    const [data, setData] = useState([]);
    const [selectedEntry, setSelectedEntry] = useState(null);

    return (
        <div className="container">
            <main>
                <div className="titleBox">
                    <div className="rex">R E X</div>
                    <div className="rexHeading">RESCUE & EXPLORATION.</div>
                </div>
                <header>
                    <h2>Operator Dashboard</h2>
                </header>
                <nav>
                    <h4>C A S E S</h4>
                </nav>
                <div className="grid-container">
                    <div className="grid">
                        <div className="grid-item">
                            <div className="widget">
                                <ThingSpeakTable setData={setData} setSelectedEntry={setSelectedEntry} />
                            </div>
                        </div>
                        <div className="grid-item">
                            <GoogleMap markersData={data} selectedEntry={selectedEntry} />
                        </div>
                    </div>
                </div>
            </main>
        </div>
    );
};

export default Dashboard;
