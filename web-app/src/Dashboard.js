import React, { useState } from 'react';
import ThingSpeakTable from './ThingSpeakTable';
import DeploymentsTable from './Deployments';
import Control from './Control';
import RobotMap from './RobotMap';
import GoogleMap from './GoogleMap';
import Gamepad from './Gamepad';
import ToggleSwitch from './ToggleSwitch';

const Dashboard = () => {
  const [data, setData] = useState([]);
  const [selectedEntry, setSelectedEntry] = useState(null);
  const [activeTab, setActiveTab] = useState('cases');
  const [isManualMode, setIsManualMode] = useState(false);

  const handleMove = (direction) => {
    console.log(`Moving ${direction}`);
    // Implement the movement logic here
  };

  const handleToggle = () => {
    setIsManualMode(!isManualMode);
  };
  console.log(data);
  const renderContent = () => {
    if (activeTab === 'cases') {
      return (
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
      );
    } else if (activeTab === 'deployments') {
      return (
        <div className="grid-container">
          <div className="grid">
            <div className="grid-item">
              <div className="widget">
                <DeploymentsTable setData={setData} setSelectedEntry={setSelectedEntry} />
              </div>
            </div>
            <div className="grid-item">
              <RobotMap markersData={data} selectedEntry={selectedEntry} />
            </div>
          </div>
        </div>
      );
    } else if (activeTab === 'control') {
      return (
        <div className="grid-container">
          <div className="grid">
            <div className="grid-item">
              <div className="widget">
                <Control setData={setData} setSelectedEntry={setSelectedEntry} />
              </div>
            </div>
            <div className="grid-item">
              <div className="widget">
                <ToggleSwitch isOn={isManualMode} handleToggle={handleToggle} />
                <Gamepad onMove={handleMove} robotName={selectedEntry ? selectedEntry.name : 'Robot Name'} />
              </div>
            </div>
          </div>
        </div>
      );
    }
  };

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
        <nav className="tabs">
          <h4
            onClick={() => setActiveTab('cases')}
            className={activeTab === 'cases' ? 'active' : ''}>
            C A S E S
          </h4>
          <h4
            onClick={() => setActiveTab('deployments')}
            className={activeTab === 'deployments' ? 'active' : ''}>
            D E P L O Y M E N T S
          </h4>
          <h4
            onClick={() => setActiveTab('control')}
            className={activeTab === 'control' ? 'active' : ''}>
            C O N T R O L
          </h4>
        </nav>
        {renderContent()}
      </main>
    </div>
  );
};

export default Dashboard;
